#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# can_data_receiver_simple.py
"""
프레이밍(SEQ/LAST) 규약을 적용하지 않는 '순수 프레임' 수신기.
- PCANManager(class)로 링크를 열고,
- PCANBasic.ReadFD를 직접 호출해 들어오는 프레임을 그대로 표시/덤프합니다.

특징:
- FD 전용(ReadFD만 사용). 클래식 Read()는 호출하지 않음.
- 포그라운드 연속 수신(기본): 버스트 드레인 루프 + 라인 버퍼링
- 백그라운드 스레드 모드(--bg), 단발 수신(--once)도 지원
"""

from __future__ import annotations

import argparse
import sys
import time
import threading
from typing import Optional, Callable

from PCANBasic import PCAN_ERROR_OK, PCAN_ERROR_QRCVEMPTY
from pcan_manager import PCANManager

try:
    from hexdump import hexdump  # 로컬 hexdump.py 있을 때 예쁜 출력
except Exception:
    hexdump = None

DEFAULT_CHANNEL = "PCAN_USBBUS1"
DEFAULT_BITRATE_FD = (
    "f_clock=80000000,"
    "nom_brp=2,nom_tseg1=33,nom_tseg2=6,nom_sjw=1,"
    "data_brp=2,data_tseg1=6,data_tseg2=1,data_sjw=1,"
    "data_ssp_offset=14"
)

# CAN FD DLC → 바이트 길이 매핑(일부 래퍼는 DLC에 '바이트'를 바로 넣기도 함)
_FD_DLC_TABLE = {
    0: 0, 1: 1, 2: 2, 3: 3, 4: 4, 5: 5, 6: 6, 7: 7, 8: 8,
    9: 12, 10: 16, 11: 20, 12: 24, 13: 32, 14: 48, 15: 64
}


def _dump_bytes(prefix: str, b: bytes, quiet: bool):
    if quiet:
        return
    if hexdump:
        print(prefix)
        hexdump(b)
    else:
        # hexdump 모듈이 없으면 간단 출력
        print(prefix, b.hex())


def _get_dlc_bytes_fd(msg) -> int:
    """
    FD 메시지의 실제 바이트 길이를 안전하게 산출.
    - 많은 래퍼가 DLC에 실제 길이를 넣지만, 표준 DLC코드(0..15)일 수도 있음.
    - DATA 배열 길이가 1..63 사이면 그 값을 신뢰.
    - 그 외에는 DLC 코드 테이블을 사용(최대 64).
    """
    try:
        data_len = len(bytes(bytearray(msg.DATA)))
    except Exception:
        data_len = 0
    if 0 < data_len <= 64:
        return data_len
    # DATA 길이로 판단이 안 될 때 DLC 사용
    try:
        dlc = int(msg.DLC)
    except Exception:
        dlc = 15  # 보수적으로 64B
    return _FD_DLC_TABLE.get(dlc, min(max(dlc, 0), 64))


def receive_once(mgr: PCANManager,
                 filter_id: Optional[int] = None,
                 timeout_s: Optional[float] = None,
                 verbose: bool = True,
                 quiet: bool = False) -> bool:
    """
    프레임 1개 수신해 출력. (프레임을 '완료'로 정의하지 않음)
    반환: 수신 여부(True=무언가 받음 / False=타임아웃)
    FD 전용(ReadFD만 사용).
    """
    deadline = None if timeout_s is None else (time.time() + float(timeout_s))

    pcan = mgr._pcan
    channel = mgr._channel
    if pcan is None or channel is None:
        raise RuntimeError("PCANManager가 열려 있지 않습니다. open() 먼저 호출하세요.")

    while True:
        # 타임아웃
        if deadline is not None and time.time() > deadline:
            if verbose and not quiet:
                print("Timeout: no frame")
            return False

        result, msg, ts = pcan.ReadFD(channel)
        if result == PCAN_ERROR_QRCVEMPTY:
            time.sleep(0.001)
            continue
        if result != PCAN_ERROR_OK:
            # 에러 텍스트 구해 출력
            try:
                _, err_txt = pcan.GetErrorText(result)
                err_str = err_txt.decode("utf-8", errors="ignore")
            except Exception:
                err_str = f"0x{int(result):X}"
            if not quiet:
                print(f"[ERR] ReadFD failed: {err_str}")
            # 에러여도 루프는 유지
            time.sleep(0.005)
            continue

        # ID 필터
        if filter_id is not None and int(msg.ID) != int(filter_id):
            continue

        # DLC 만큼만 덤프
        dlc = _get_dlc_bytes_fd(msg)
        raw = bytes(bytearray(msg.DATA)[:dlc])

        if not quiet:
            print(f"RX(FD): ID=0x{int(msg.ID):X} DLC={dlc}")
        _dump_bytes("FRAME:", raw, quiet=quiet)
        return True


def loop_foreground(mgr,
                    filter_id: Optional[int] = None,
                    quiet: bool = False,
                    stats: Optional[float] = None,
                    duration: Optional[float] = None,
                    yield_ms: int = 1) -> None:
    """
    ReadFD를 '버스트 드레인' 방식으로 계속 호출하여 연속 수신.
    filter_id가 None이면 모든 ID 출력.
    FD 전용(ReadFD만 사용).
    """
    import sys as _sys

    pcan = mgr._pcan
    ch = mgr._channel
    if pcan is None or ch is None:
        raise RuntimeError("PCANManager가 열려 있지 않습니다. open() 먼저 호출하세요.")

    # PyCharm 콘솔에서도 줄 단위로 바로바로 보이게
    try:
        _sys.stdout.reconfigure(line_buffering=True)  # Python 3.7+
    except Exception:
        pass

    print("Receiving frames... (Ctrl+C to stop)")
    rx_count = 0
    start_ts = last_stat_ts = time.time()

    try:
        while True:
            any_rx = False

            # === 큐가 빌 때까지 한 번에 다 뽑아 처리 ===
            while True:
                result, msg, ts = pcan.ReadFD(ch)

                if result == PCAN_ERROR_QRCVEMPTY:
                    break  # 이번 턴 드레인 완료

                if result != PCAN_ERROR_OK:
                    if not quiet:
                        try:
                            _, err_txt = pcan.GetErrorText(result)
                            err_str = err_txt.decode("utf-8", errors="ignore")
                        except Exception:
                            err_str = f"0x{int(result):X}"
                        print(f"[ERR] ReadFD failed: {err_str}")
                    time.sleep(0.005)
                    continue

                can_id = int(msg.ID)
                if (filter_id is None) or (can_id == int(filter_id)):
                    dlc = _get_dlc_bytes_fd(msg)
                    raw = bytes(bytearray(msg.DATA)[:dlc])
                    if not quiet:
                        print(f"RX(FD): ID=0x{can_id:X} DLC={dlc}")
                        _dump_bytes("FRAME:", raw, quiet=quiet)
                        _sys.stdout.flush()
                    rx_count += 1
                    any_rx = True
                # 필터 불일치 프레임은 조용히 스킵

            # 큐가 비어 있었으면 잠깐 쉼 (CPU 100% 방지)
            if not any_rx:
                time.sleep(max(0, yield_ms) / 1000.0)

            # 통계/종료 관리
            now = time.time()
            if stats and (now - last_stat_ts) >= float(stats):
                elapsed = now - start_ts
                rate = rx_count / max(1e-9, elapsed)
                print(f"[STATS] frames={rx_count} elapsed={elapsed:.2f}s rate={rate:.1f} fps")
                last_stat_ts = now
            if (duration is not None) and (now - start_ts >= float(duration)):
                break

    except KeyboardInterrupt:
        print("\n[INFO] Stopped by user.")


# =========================
#   Continuous Receiver (BG)
# =========================
class ContinuousReceiver:
    """
    PCANBasic.ReadFD를 루프/스레드로 돌면서 연속 수신해
    on_frame 콜백 또는 콘솔로 표시합니다.
    FD 전용(ReadFD만 사용).
    """
    def __init__(self,
                 mgr: PCANManager,
                 filter_id: Optional[int] = None,
                 on_frame: Optional[Callable[[int, int, bytes], None]] = None,
                 quiet: bool = False,
                 idle_sleep_s: float = 0.001):
        self.mgr = mgr
        self.filter_id = filter_id
        self.on_frame = on_frame
        self.quiet = quiet
        self.idle_sleep_s = idle_sleep_s

        self._run = False
        self._th: Optional[threading.Thread] = None

    def _loop_once(self) -> bool:
        pcan = self.mgr._pcan
        channel = self.mgr._channel
        if pcan is None or channel is None:
            raise RuntimeError("PCANManager가 열려 있지 않습니다. open() 먼저 호출하세요.")

        result, msg, ts = pcan.ReadFD(channel)
        if result == PCAN_ERROR_QRCVEMPTY:
            time.sleep(self.idle_sleep_s)
            return False

        if result != PCAN_ERROR_OK:
            try:
                _, err_txt = pcan.GetErrorText(result)
                err_str = err_txt.decode("utf-8", errors="ignore")
            except Exception:
                err_str = f"0x{int(result):X}"
            if not self.quiet:
                print(f"[ERR] ReadFD failed: {err_str}")
            time.sleep(0.005)
            return False

        can_id = int(msg.ID)
        if self.filter_id is not None and can_id != int(self.filter_id):
            return False

        dlc = _get_dlc_bytes_fd(msg)
        raw = bytes(bytearray(msg.DATA)[:dlc])

        if self.on_frame:
            try:
                self.on_frame(can_id, dlc, raw)
            except Exception as e:
                if not self.quiet:
                    print(f"[WARN] on_frame error: {e}")
        else:
            if not self.quiet:
                print(f"RX(FD): ID=0x{can_id:X} DLC={dlc}")
            _dump_bytes("FRAME:", raw, quiet=self.quiet)

        return True

    def start(self):
        if self._run:
            return
        self._run = True
        self._th = threading.Thread(target=self._thread_main, name="SimpleRxFD", daemon=True)
        self._th.start()

    def _thread_main(self):
        while self._run:
            try:
                self._loop_once()
            except Exception as e:
                if not self.quiet:
                    print(f"[ERR] Rx loop error: {e}")
                time.sleep(0.01)

    def stop(self, join_timeout_s: float = 0.5):
        self._run = False
        th = self._th
        self._th = None
        if th and th.is_alive():
            try:
                th.join(timeout=join_timeout_s)
            except Exception:
                pass


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="Simple CAN-FD frame receiver (no framing), via PCANManager (FD only)")
    ap.add_argument("--channel", default=DEFAULT_CHANNEL, help="PCAN channel (e.g., PCAN_USBBUS1)")
    ap.add_argument("--bitrate-fd", default=DEFAULT_BITRATE_FD, help="FD bitrate string")
    ap.add_argument("--ifg-us", type=int, default=3000, help="inter-frame gap (us) for TX; RX에는 영향 없음")
    ap.add_argument("--filter-id", type=lambda x: int(x, 0), default=None, help="only print frames for this CAN ID")
    ap.add_argument("--timeout", type=float, default=None, help="timeout seconds (for single receive)")
    # ---- 모드 선택 (아무 것도 안 주면 loop가 기본) ----
    mx = ap.add_mutually_exclusive_group()
    mx.add_argument("--loop", action="store_true", help="foreground continuous receiving (default)")
    mx.add_argument("--bg", action="store_true", help="background thread continuous receiving")
    mx.add_argument("--once", action="store_true", help="receive a single frame and exit")
    ap.add_argument("--duration", type=float, default=None, help="auto-stop after N seconds (loop/bg)")
    ap.add_argument("--stats", type=float, default=None, help="print RX stats every N seconds")
    ap.add_argument("--quiet", action="store_true", help="suppress frame hex dumps and logs")
    ap.add_argument("--no-verbose", action="store_true", help="(legacy) suppress extra prints in receive_once")
    args = ap.parse_args(argv)

    # 기본 모드: 아무 것도 지정 안 하면 loop
    mode = "loop"
    if args.bg:
        mode = "bg"
    elif args.once:
        mode = "once"
    elif args.loop:
        mode = "loop"

    mgr = PCANManager()
    try:
        # FD 모드로 오픈 (pcan_manager는 InitializeFD/BitrateFD 세팅을 래핑한다고 가정)
        mgr.open(args.channel, args.bitrate_fd, ifg_us=int(args.ifg_us))

        if mode == "bg":
            # 백그라운드 연속 수신
            rx_count = 0
            last_stat_ts = time.time()

            def _on_frame(can_id: int, dlc: int, raw: bytes):
                nonlocal rx_count
                rx_count += 1
                if not args.quiet:
                    print(f"RX(FD): ID=0x{can_id:X} DLC={dlc}")
                    _dump_bytes("FRAME:", raw, quiet=args.quiet)

            rx = ContinuousReceiver(mgr=mgr, filter_id=args.filter_id, on_frame=_on_frame, quiet=args.quiet)
            rx.start()
            print("Receiving (background thread). Press Ctrl+C to stop."
                  if args.duration is None else f"Receiving (background thread) for {args.duration:.1f}s...")
            try:
                start_ts = time.time()
                while True:
                    time.sleep(0.1)
                    if args.stats:
                        now = time.time()
                        if now - last_stat_ts >= float(args.stats):
                            elapsed = now - start_ts
                            rate = rx_count / max(1e-9, elapsed)
                            print(f"[STATS] frames={rx_count} elapsed={elapsed:.2f}s rate={rate:.1f} fps")
                            last_stat_ts = now
                    if args.duration is not None and (time.time() - start_ts) >= float(args.duration):
                        break
            except KeyboardInterrupt:
                print("\n[INFO] Stopped by user.")
            finally:
                rx.stop()

        elif mode == "loop":
            # ✅ 기본: 포그라운드 연속 수신 (FD 버스트 드레인)
            loop_foreground(
                mgr,
                filter_id=args.filter_id,
                quiet=args.quiet,
                stats=args.stats,
                duration=args.duration,
                yield_ms=1
            )

        else:  # once
            _ = receive_once(
                mgr,
                filter_id=args.filter_id,
                timeout_s=args.timeout,
                verbose=(not args.no_verbose),
                quiet=args.quiet,
            )

    finally:
        mgr.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
