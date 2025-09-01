#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
can_custom_tp_receiver.py

프레이밍 규약:
  - frame[0] = HDR (1B)
      * bit7 = 0 → MIDDLE 프레임, bit6..0 = SEQ (0..127)
      * bit7 = 1 → LAST   프레임, bit6..0 = last_len (1..63)
  - frame[1:64] = DATA 63B (LAST에서는 last_len 만큼 유효, 나머지 0 패딩)

기능:
  - PCANManager로부터 CAN FD 프레임(최대 64B)을 받아 위 규약에 따라 조립
  - CAN ID별 동시 조립 상태 관리
  - 콜백 기반(start_rx) 또는 폴링 기반(start_polling/receive_packet) 지원
  - **완성 패킷 조립 시 hexdump 콘솔 출력(코드 검증용)**

단독 실행 예:
  python can_custom_tp_receiver.py --channel PCAN_USBBUS1 --bitrate-fd "..." --filter-ids "0xD1,0x200-0x20F"
"""

from __future__ import annotations
from typing import Optional, Iterable, Tuple, Set, Callable, Any, Dict
import threading
import time
import queue
import argparse
import re
import sys

try:
    from pcan_manager import PCANManager
except Exception as e:
    PCANManager = None  # type: ignore
    _pcan_import_err = e


# ---------- 내부 유틸 ----------
_DLC_TO_LEN = {
    0x0: 0,  0x1: 1,  0x2: 2,  0x3: 3,
    0x4: 4,  0x5: 5,  0x6: 6,  0x7: 7,
    0x8: 8,  0x9: 12, 0xA: 16, 0xB: 20,
    0xC: 24, 0xD: 32, 0xE: 48, 0xF: 64,
}

def _length_from_msgfd(msg: Any) -> int:
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
            return min(len(msg.DATA), 64)
        except Exception:
            pass
    return 64


# ---------- 조립 상태 ----------
class _AsmState:
    __slots__ = ("buf", "started", "last_seq", "last_ts")
    def __init__(self):
        self.buf = bytearray()
        self.started = False
        self.last_seq: Optional[int] = None
        self.last_ts = time.time()

    def reset(self):
        self.buf.clear()
        self.started = False
        self.last_seq = None
        self.touch()

    def touch(self):
        self.last_ts = time.time()


# ---------- 메인 클래스 ----------
class CanCustomTpReceiver:
    """
    완성 메시지 수신기 (프레이밍 조립 포함)

    Args:
        mgr: PCANManager 인스턴스 (open() 완료 권장)
        filter_can_id: 단일 CAN ID 필터 (옵션)
        filter_can_ids: 여러 CAN ID 필터(set, 옵션). 둘 다 주면 합집합.
        assembly_timeout_s: 조립 중 inactivity 시 리셋 타임아웃(초)
    """
    def __init__(self,
                 mgr: "PCANManager",
                 filter_can_id: Optional[int] = None,
                 filter_can_ids: Optional[Iterable[int]] = None,
                 assembly_timeout_s: float = 2.0):
        if PCANManager is None:
            raise RuntimeError(f"pcan_manager import 실패: {_pcan_import_err}")
        self._mgr = mgr

        fs: Set[int] = set()
        if filter_can_ids:
            fs.update(int(x) for x in filter_can_ids)
        if filter_can_id is not None:
            fs.add(int(filter_can_id))
        self._filters: Optional[Set[int]] = fs or None  # None => 전체 수신

        self._asm: Dict[int, _AsmState] = {}
        self._asm_timeout = float(assembly_timeout_s)

        # 콜백/스레드/큐
        self._on_packet: Optional[Callable[[bytes, int], None]] = None
        self._rx_thread: Optional[threading.Thread] = None
        self._rx_stop = threading.Event()
        self._complete_q: "queue.Queue[Tuple[bytes, int]]" = queue.Queue()

    # ===== 퍼블릭 API =====
    def close(self):
        self.stop_rx()

    def start_rx(self, on_packet: Optional[Callable[[bytes, int], None]] = None):
        """
        콜백 기반 연속 수신 시작.
        - 가능하면 mgr.start_rx(on_frame=cb) 사용
        - 실패하면 내부 폴링 스레드로 대체
        """
        if self._rx_thread and self._rx_thread.is_alive():
            return
        self._on_packet = on_packet
        self._rx_stop.clear()

        # 1) PCANManager의 콜백이 있으면 사용
        if hasattr(self._mgr, "start_rx") and hasattr(self._mgr, "stop_rx"):
            try:
                def _cb(payload: bytes, can_id: int):
                    self._handle_raw_frame(int(can_id), bytes(payload))
                try:
                    self._mgr.start_rx(_cb)  # type: ignore
                    return
                except TypeError:
                    self._mgr.start_rx(on_frame=_cb)  # type: ignore
                    return
            except Exception:
                pass

        def _loop():
            while not self._rx_stop.is_set():
                try:
                    msg = self._read_once()
                    if msg is None:
                        time.sleep(0.001)
                        continue
                    can_id, data = msg
                    self._handle_raw_frame(can_id, data)
                except Exception:
                    time.sleep(0.010)

        self._rx_thread = threading.Thread(target=_loop, name="CanRx-Assemble", daemon=True)
        self._rx_thread.start()

    def stop_rx(self):
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

    def start_polling(self, poll_interval_s: float = 0.001):
        if self._rx_thread and self._rx_thread.is_alive():
            return
        self._rx_stop.clear()

        def _loop():
            while not self._rx_stop.is_set():
                try:
                    msg = self._read_once()
                    if msg is None:
                        time.sleep(poll_interval_s)
                        self._gc_stale()
                        continue
                    can_id, data = msg
                    self._handle_raw_frame(can_id, data)
                except Exception:
                    time.sleep(0.010)

        self._rx_thread = threading.Thread(target=_loop, name="CanRx-AssemblePoll", daemon=True)
        self._rx_thread.start()

    def stop_polling(self):
        self.stop_rx()

    def receive_packet(self, timeout_s: float = 1.0, verbose: bool = False) -> Optional[Tuple[bytes, int]]:
        """
        조립 완료 패킷을 (data, can_id)로 반환. 타임아웃 시 None.
        start_rx/start_polling 여부와 무관하게 사용할 수 있음.
        """
        try:
            return self._complete_q.get(timeout=max(0.0, float(timeout_s)))
        except queue.Empty:
            return None

    # ===== 내부 처리 =====
    def _pass_filter(self, can_id: int) -> bool:
        return (self._filters is None) or (can_id in self._filters)

    @staticmethod
    def _hexdump_print(can_id: int, payload: bytes):
        ts = time.strftime("%H:%M:%S")
        n = len(payload)
        print(f"[{ts}] [RX-COMP] id=0x{can_id:X}, len={n}", flush=True)
        for off in range(0, n, 16):
            chunk = payload[off:off+16]
            hexpart = " ".join(f"{b:02X}" for b in chunk)
            ascii_part = "".join(chr(b) if 32 <= b <= 126 else "." for b in chunk)
            print(f"{off:04X}  {hexpart:<47}  {ascii_part}", flush=True)
        if n == 0:
            print("0000", flush=True)
        print("", flush=True)

    def _handle_raw_frame(self, can_id: int, frame: bytes):
        """
        수신한 "단일 CAN 프레임"을 프레이밍 규약에 맞춰 조립 처리.
        frame: 최대 64B (frame[0]=HDR, frame[1:]=DATA 최대 63B)
        """
        if not self._pass_filter(can_id):
            return
        if not frame:
            return

        hdr = frame[0]
        data = frame[1:64]

        is_last = bool(hdr & 0x80)
        low7 = hdr & 0x7F

        st = self._asm.get(can_id)
        if st is None:
            st = _AsmState()
            self._asm[can_id] = st

        st.touch()

        if is_last:
            last_len = low7
            if last_len == 0 or last_len > 63:
                st.reset()
                return
            tail = data[:last_len]

            if not st.started:
                # 단일 LAST → 바로 완성 (hexdump 출력)
                payload = bytes(tail)
                self._hexdump_print(can_id, payload)
                self._on_completed(can_id, payload)
                st.reset()
                return
            else:
                st.buf += tail
                payload = bytes(st.buf)
                self._hexdump_print(can_id, payload)
                self._on_completed(can_id, payload)
                st.reset()
                return

        # MIDDLE
        seq = low7  # 0..127
        if (not st.started) or (seq == 0):
            st.buf.clear()
            st.started = True
            st.last_seq = seq
        else:
            if st.last_seq is not None:
                exp = (st.last_seq + 1) & 0x7F
                if seq != exp:
                    st.buf.clear()
                    st.started = True
            st.last_seq = seq

        st.buf += data
        self._gc_stale_one(can_id, st)

    def _on_completed(self, can_id: int, payload: bytes):
        try:
            self._complete_q.put_nowait((payload, can_id))
        except Exception:
            pass
        if self._on_packet:
            try:
                self._on_packet(payload, can_id)
            except Exception:
                pass

    def _gc_stale(self):
        now = time.time()
        drop_keys = [
            cid for cid, st in self._asm.items()
            if (now - st.last_ts) > self._asm_timeout
        ]
        for cid in drop_keys:
            try:
                self._asm[cid].reset()
            except Exception:
                pass
            self._asm.pop(cid, None)

    def _gc_stale_one(self, can_id: int, st: _AsmState):
        if (time.time() - st.last_ts) > self._asm_timeout:
            st.reset()

    # ---- mgr로부터 한 번 읽기 ----
    def _read_once(self) -> Optional[Tuple[int, bytes]]:
        """
        가능한 PCANManager API를 순서대로 시도해 1프레임 읽기.
        성공 시 (can_id, raw_frame_bytes) 반환.
        """
        for name in ("read_once", "read_fd", "readFD", "read", "receive"):
            if hasattr(self._mgr, name):
                fn = getattr(self._mgr, name)
                try:
                    obj = fn()
                except TypeError:
                    try:
                        obj = fn(0)  # non-block
                    except Exception:
                        obj = None
                except Exception:
                    obj = None

                if obj is None:
                    continue
                msg = obj[0] if isinstance(obj, tuple) else obj
                can_id, data = self._extract_from_msgfd(msg)
                if can_id is None or data is None:
                    continue
                return can_id, data
        return None

    def _extract_from_msgfd(self, msg: Any) -> Tuple[Optional[int], Optional[bytes]]:
        try:
            can_id = int(msg.ID)
        except Exception:
            return None, None

        n = _length_from_msgfd(msg)
        n = max(0, min(64, n))
        try:
            raw = msg.DATA[:n]
            data = bytes(int(x) & 0xFF for x in raw)
        except Exception:
            try:
                data = bytes(msg.DATA)[:n]
            except Exception:
                return None, None

        return can_id, data


# ---------- 유틸: 필터 문자열 파서 ----------
def _parse_filter_ids(text: str) -> Optional[Set[int]]:
    """
    입력 예:
      ""                   -> None (전체 수신)
      "0xD1"               -> {0xD1}
      "0xD1,0xD2 0xD3"     -> {0xD1, 0xD2, 0xD3}
      "0x100-0x10F"        -> {0x100..0x10F}
      "0xD1, 0x200-0x205"  -> 혼합 가능
    """
    s = (text or "").strip()
    if not s:
        return None
    ids: Set[int] = set()
    for tok in re.split(r"[,\s]+", s):
        if not tok:
            continue
        if "-" in tok:
            a, b = tok.split("-", 1)
            lo = int(a, 0); hi = int(b, 0)
            if lo > hi:
                lo, hi = hi, lo
            ids.update(range(lo, hi + 1))
        else:
            ids.add(int(tok, 0))
    return ids or None


# ---------- main ----------
def main():
    if PCANManager is None:
        print(f"[ERR] pcan_manager import 실패: {_pcan_import_err}", file=sys.stderr)
        sys.exit(2)

    parser = argparse.ArgumentParser(description="CAN data receiver (HDR+DATA[63] framing, hexdump on complete)")
    parser.add_argument("--channel", default="PCAN_USBBUS1", help="PCAN channel (default: PCAN_USBBUS1)")
    parser.add_argument(
        "--bitrate-fd",
        default=(
            "f_clock=80000000,nom_brp=2,nom_tseg1=33,nom_tseg2=6,nom_sjw=1,"
            "data_brp=2,data_tseg1=6,data_tseg2=1,data_sjw=1,data_ssp_offset=14"
        ),
        help="PCAN-FD bitrate string"
    )
    parser.add_argument("--ifg-us", type=int, default=3000, help="Inter-frame gap for TX (us); passed to manager open")
    parser.add_argument("--filter-ids", default="", help='예: "0xD1,0xD2 0x300-0x30F" (공란이면 전체)')
    parser.add_argument("--mode", choices=["callback", "poll"], default="callback", help="수신 모드")
    parser.add_argument("--timeout-s", type=float, default=1.0, help="poll 모드에서 receive_packet 타임아웃")
    parser.add_argument("--asm-timeout-s", type=float, default=2.0, help="조립 타임아웃(초)")
    args = parser.parse_args()

    mgr = PCANManager()
    try:
        mgr.open(args.channel, args.bitrate_fd, ifg_us=int(args.ifg_us))
    except Exception as e:
        print(f"[ERR] PCAN open 실패: {e}", file=sys.stderr)
        sys.exit(3)

    filter_ids = _parse_filter_ids(args.filter_ids)
    rx = CanCustomTpReceiver(mgr, filter_can_ids=filter_ids, assembly_timeout_s=float(args.asm_timeout_s))

    print("[INFO] CAN Receiver 시작", flush=True)
    print(f"       channel={args.channel}", flush=True)
    print(f"       bitrate_fd={args.bitrate_fd}", flush=True)
    print(f"       filter_ids={'ALL' if not filter_ids else ','.join(hex(i) for i in sorted(filter_ids))}", flush=True)
    print(f"       mode={args.mode}", flush=True)
    print("Receiving... (Ctrl+C to stop)\n", flush=True)

    try:
        if args.mode == "callback":
            def _on_pkt(_data: bytes, _cid: int):
                pass
            rx.start_rx(on_packet=_on_pkt)
            while True:
                time.sleep(0.2)
        else:
            while True:
                pkt = rx.receive_packet(timeout_s=float(args.timeout_s))
                _ = pkt
    except KeyboardInterrupt:
        print("\n[INFO] Stopping...", flush=True)
    except Exception as e:
        print(f"[ERR] {e}", file=sys.stderr, flush=True)
    finally:
        try:
            rx.close()
        except Exception:
            pass
        try:
            if hasattr(mgr, "close"):
                mgr.close()
        except Exception:
            pass
        print("[INFO] Stopped.", flush=True)

if __name__ == "__main__":
    main()
