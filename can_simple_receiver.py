#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
can_simple_receiver.py

- PCANManager를 공용으로 사용하여 CAN FD 프레임(최대 64B) 연속 수신
- 수신 시 hexdump 프린트 + 파일 저장(패킷 간 1줄 공백)
- 다중 CAN ID 필터 지원(미지정(None) 시 전체 수신)
- 콜백 기반(mgr.start_rx) 지원, 없으면 내부 폴링 스레드
- 실행 시마다 "radar packet YYYY-MM-DD HH-mm-ss.log" 생성

필요:
    - pcan_manager.py (repo 기준, 동작 확인된 버전)
"""

from __future__ import annotations
from typing import Optional, Iterable, Tuple, Set, Callable, Any

import os
import re
import sys
import time
import threading
from datetime import datetime

try:
    from pcan_manager import PCANManager
except Exception as e:
    PCANManager = None  # type: ignore
    _pcan_import_err = e


# -------------------- 유틸 --------------------
_DLC_TO_LEN = {
    0x0: 0,  0x1: 1,  0x2: 2,  0x3: 3,
    0x4: 4,  0x5: 5,  0x6: 6,  0x7: 7,
    0x8: 8,  0x9: 12, 0xA: 16, 0xB: 20,
    0xC: 24, 0xD: 32, 0xE: 48, 0xF: 64,
}

def _length_from_msgfd(msg: Any) -> int:
    # PCANBasic TPCANMsgFD는 구현에 따라 LEN이 없고 DLC만 있을 수 있음
    if hasattr(msg, "LEN"):
        try:
            n = int(msg.LEN)
            if 0 <= n <= 64:
                return n
        except Exception:
            pass
    if hasattr(msg, "DLC"):
        try:
            dlc = int(msg.DLC) & 0xF
            return _DLC_TO_LEN.get(dlc, 64)
        except Exception:
            pass
    if hasattr(msg, "DATA"):
        try:
            # ctypes 배열 길이 또는 파이썬 bytes/bytearray 길이
            return min(len(msg.DATA), 64)
        except Exception:
            pass
    return 64

def _hexdump_lines(b: bytes) -> str:
    lines = []
    for i in range(0, len(b), 16):
        chunk = b[i:i+16]
        hexs = " ".join(f"{x:02X}" for x in chunk)
        asci = "".join(chr(x) if 32 <= x <= 126 else "." for x in chunk)
        lines.append(f"{i:04X}  {hexs:<47}  {asci}")
    return "\n".join(lines)

def _sanitize_windows_filename(name: str) -> str:
    return re.sub(r'[<>:"/\\|?*]', '-', name)

def _new_log_path(prefix: str, log_dir: str = ".") -> str:
    ts = datetime.now().strftime("%Y-%m-%d %H-%M-%S")  # 파일명 안전형
    return os.path.join(log_dir, _sanitize_windows_filename(f"{prefix} {ts}.log"))

def _nonzero(b: bytes) -> bool:
    # 전부 0x00이면 False
    for x in b:
        if x:
            return True
    return False


# -------------------- 메인 클래스 --------------------
class CanSimpleReceiver:
    """
    간단 수신기:
      - mgr: PCANManager 인스턴스 (open() 완료 상태 권장)
      - filter_can_id / filter_can_ids: 소프트 필터(공란/None이면 전체 수신)
      - log_path: 지정하면 즉시 열고, None이면 자동("radar packet ...")
    """
    def __init__(self,
                 mgr: PCANManager,
                 filter_can_id: Optional[int] = None,
                 filter_can_ids: Optional[Iterable[int]] = None,
                 log_path: Optional[str] = None):
        if PCANManager is None:
            raise RuntimeError(f"pcan_manager import 실패: {_pcan_import_err}")
        self._mgr = mgr
        s: Set[int] = set()
        if filter_can_ids:
            s.update(int(x) for x in filter_can_ids)
        if filter_can_id is not None:
            s.add(int(filter_can_id))
        self._filters: Optional[Set[int]] = s or None  # None -> 전체 허용

        self._rx_thread: Optional[threading.Thread] = None
        self._rx_stop = threading.Event()
        self._on_packet: Optional[Callable[[bytes, int], None]] = None

        # 로그 파일
        if log_path is None:
            log_path = _new_log_path("radar packet", ".")
        self._log_path = log_path
        try:
            self._log_fp = open(self._log_path, "w", encoding="utf-8", newline="")
        except Exception:
            self._log_fp = None

    # ---------- 퍼블릭 API ----------
    def close(self):
        self.stop_rx()
        try:
            if self._log_fp:
                self._log_fp.flush()
                self._log_fp.close()
        finally:
            self._log_fp = None

    def start_rx(self, on_packet: Optional[Callable[[bytes, int], None]] = None):
        """
        콜백 기반 연속 수신 시작.
        PCANManager가 자체 start_rx를 제공하면 그걸 사용하고,
        아니면 내부 스레드 폴링로 대체.
        """
        if self._rx_thread and self._rx_thread.is_alive():
            return
        self._on_packet = on_packet
        self._rx_stop.clear()

        # 1) PCANManager가 자체 콜백을 제공하는 경우 사용
        if hasattr(self._mgr, "start_rx") and hasattr(self._mgr, "stop_rx"):
            try:
                def _cb(payload: bytes, can_id: int):
                    self._handle_frame(int(can_id), bytes(payload))
                # start_rx 시그니처 유연 처리
                try:
                    self._mgr.start_rx(_cb)  # type: ignore
                    return
                except TypeError:
                    self._mgr.start_rx(on_frame=_cb)  # type: ignore
                    return
            except Exception:
                # 실패하면 폴링으로 폴백
                pass

        # 2) 내부 폴링 스레드
        def _loop():
            while not self._rx_stop.is_set():
                try:
                    msg = self._read_once()
                    if msg is None:
                        time.sleep(0.001)
                        continue
                    can_id, data = msg
                    self._handle_frame(can_id, data)
                except Exception:
                    time.sleep(0.010)
        self._rx_thread = threading.Thread(target=_loop, name="CanRxSimple", daemon=True)
        self._rx_thread.start()

    def stop_rx(self):
        # PCANManager 기반이면 그것도 정지 시도
        try:
            if hasattr(self._mgr, "stop_rx"):
                self._mgr.stop_rx()  # type: ignore
        except Exception:
            pass

        self._rx_stop.set()
        if self._rx_thread and self._rx_thread.is_alive():
            try:
                self._rx_thread.join(timeout=1.0)
            except Exception:
                pass
        self._rx_thread = None

    def receive_packet(self, timeout_s: float = 1.0, verbose: bool = False) -> Optional[Tuple[bytes, int]]:
        """
        단발 수신(폴링). 타임아웃 내 수신 없으면 None.
        """
        t0 = time.time()
        while True:
            msg = self._read_once()
            if msg is not None:
                can_id, data = msg
                if self._pass_filter(can_id) and _nonzero(data):
                    # 옵션: 콘솔 프린트
                    if verbose:
                        self._print_packet(can_id, data)
                    # 파일 저장
                    self._save_packet(can_id, data)
                    return data, can_id
            if (time.time() - t0) >= timeout_s:
                return None
            time.sleep(0.001)

    # ---------- 내부 처리 ----------
    def _pass_filter(self, can_id: int) -> bool:
        return (self._filters is None) or (can_id in self._filters)

    def _handle_frame(self, can_id: int, data: bytes):
        # 필터/제로 프레임 처리
        if not self._pass_filter(can_id):
            return
        if not _nonzero(data):
            return

        # 콘솔 출력(hexdump)
        self._print_packet(can_id, data)
        # 파일 저장
        self._save_packet(can_id, data)
        # 상위 콜백
        if self._on_packet:
            try:
                self._on_packet(data, can_id)
            except Exception:
                pass

    def _print_packet(self, can_id: int, data: bytes):
        ts = datetime.now().strftime("%H:%M:%S")
        sys.stdout.write(f"[{ts}] CAN ID: 0x{can_id:X}\n")
        sys.stdout.write(_hexdump_lines(data) + "\n\n")
        sys.stdout.flush()

    def _save_packet(self, can_id: int, data: bytes):
        if not self._log_fp:
            return
        try:
            self._log_fp.write(f"CAN ID: 0x{can_id:X}\n")
            self._log_fp.write(data.hex().upper() + "\n\n")  # 패킷 사이 공백 줄
            self._log_fp.flush()
        except Exception:
            pass

    # 다양한 PCANManager 구현을 호환하기 위한 단일 read
    def _read_once(self) -> Optional[Tuple[int, bytes]]:
        """
        가능한 매니저 API를 순서대로 시도해 한 번 읽기.
        성공 시 (can_id, payload64B) 반환, 없으면 None.
        """
        # 1) mgr.read_once() → (msg,) or msg
        for name in ("read_once", "read_fd", "readFD", "read", "receive"):
            if hasattr(self._mgr, name):
                fn = getattr(self._mgr, name)
                try:
                    obj = fn()
                except TypeError:
                    # 일부 구현은 timeout_ms나 채널을 요구할 수 있음 → 기본값 시도
                    try:
                        obj = fn(0)  # non-block
                    except Exception:
                        obj = None
                except Exception:
                    obj = None

                if obj is None:
                    continue
                # obj가 (msg, ts) 형태일 수 있음
                msg = obj[0] if isinstance(obj, tuple) else obj
                can_id, data = self._extract_from_msgfd(msg)
                if can_id is None or data is None:
                    continue
                return can_id, data

        # 2) 하위 필드 접근이 가능한 경우 (최후 수단) → 실패 시 None
        return None

    def _extract_from_msgfd(self, msg: Any) -> Tuple[Optional[int], Optional[bytes]]:
        try:
            can_id = int(msg.ID)
        except Exception:
            return None, None

        # 길이 계산
        n = _length_from_msgfd(msg)
        n = max(0, min(64, n))
        # 데이터 추출
        data: bytes
        try:
            # msg.DATA가 ctypes 배열인 경우
            raw = msg.DATA[:n]
            data = bytes(int(x) & 0xFF for x in raw)
        except Exception:
            try:
                data = bytes(msg.DATA)
                data = data[:n]
            except Exception:
                return None, None

        # 수신 규칙: 고정 64B로 운용한다면, 길이가 짧아도 64로 맞출 수 있음
        # 여기서는 실제 길이 n만 사용(저장/표시 모두 n바이트)
        return can_id, data


# -------------------- 실행 진입점 --------------------
def main():
    """
    간단 실행:
        python can_custom_receiver_simple.py
    환경변수:
        PCAN_CHANNEL   (기본: PCAN_USBBUS1)
        PCAN_BITRATEFD (기본: 레포 예제값)
        PCAN_IFG_US    (기본: 3000)
        PCAN_FILTER    (예: "0xD1,0xD2 0x100-0x10F" → 다중/범위)
    """
    if PCANManager is None:
        print(f"[ERR] pcan_manager import 실패: {_pcan_import_err}", file=sys.stderr)
        sys.exit(1)

    channel = os.environ.get("PCAN_CHANNEL", "PCAN_USBBUS1")
    bitrate_fd = os.environ.get(
        "PCAN_BITRATEFD",
        "f_clock=80000000,nom_brp=2,nom_tseg1=33,nom_tseg2=6,nom_sjw=1,"
        "data_brp=2,data_tseg1=6,data_tseg2=1,data_sjw=1,data_ssp_offset=14"
    )
    ifg_us = int(os.environ.get("PCAN_IFG_US", "3000"))

    # 필터 파싱
    filt_env = os.environ.get("PCAN_FILTER", "").strip()
    filter_ids: Optional[Set[int]] = None
    if filt_env:
        ids: Set[int] = set()
        for tok in re.split(r"[,\s]+", filt_env):
            if not tok:
                continue
            if "-" in tok:
                a, b = tok.split("-", 1)
                try:
                    lo = int(a, 0); hi = int(b, 0)
                    if lo > hi:
                        lo, hi = hi, lo
                    ids.update(range(lo, hi + 1))
                except Exception:
                    pass
            else:
                try:
                    ids.add(int(tok, 0))
                except Exception:
                    pass
        filter_ids = ids or None

    # 매니저 오픈
    mgr = PCANManager()
    mgr.open(channel, bitrate_fd, ifg_us=ifg_us)

    # 수신기 생성
    rx = CanSimpleReceiver(mgr, filter_can_ids=filter_ids)

    print("Receiving frames continuously... (Ctrl+C to stop)")

    try:
        # 콜백 기반 (가능하면)
        def _on_packet(data: bytes, can_id: int):
            # 실제 처리(프린트/저장)는 클래스 내부에서 이미 수행됨.
            # 여기선 추가 가공이 필요하면 작성.
            pass

        try:
            rx.start_rx(on_packet=_on_packet)
            # 메인 스레드 idle 대기
            while True:
                time.sleep(0.2)
        except Exception:
            # 콜백이 불가하면 폴링
            while True:
                rx.receive_packet(timeout_s=0.5, verbose=True)

    except KeyboardInterrupt:
        print("Stopped by user.")
    finally:
        try:
            rx.stop_rx()
        except Exception:
            pass
        try:
            rx.close()
        except Exception:
            pass
        try:
            if hasattr(mgr, "close"):
                mgr.close()
        except Exception:
            pass


if __name__ == "__main__":
    main()
