#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
can_data_receiver.py

프레이밍 규약(64B 고정, FD DLC=15)
  - frame[0] = HDR (1B)
      * bit7 = 0 → MIDDLE 프레임, bit6..0 = SEQ (0..127)
      * bit7 = 1 → LAST   프레임, bit6..0 = last_len (1..63)
  - frame[1:64] = DATA 63B (LAST에서는 last_len 만큼 유효, 나머지 0 패딩)

기능:
  - CanDataReceiver: start_rx/stop_rx, start_polling/stop_polling, receive_packet, receiveData, close
  - main(): 모의 PCAN 또는 실제 PCAN으로 수신 검증

사용 예:
  # 모의 PCAN으로 10초간 콜백 기반 검증 (기본)
  python can_data_receiver.py

  # 모의 PCAN, 폴링 기반 + receive_packet()으로 검증
  python can_data_receiver.py --poll --duration 5

  # 실제 PCAN으로 콜백 기반 수신 검증
  python can_data_receiver.py --mode pcan --channel PCAN_USBBUS1 --filter-id 0xC0

  # 조용한 로그 (프레임/패킷 hexdump 끄기)
  python can_data_receiver.py --quiet
"""

from __future__ import annotations
from typing import Optional, Tuple, Dict, Callable, Any, List
import time
import threading
from queue import Queue, Empty
import argparse
import binascii
import sys

# ============= hexdump 안전 임포트 & 프리뷰 유틸 =============
try:
    from hexdump import hexdump as _hexdump  # type: ignore
except Exception:
    _hexdump = None

def _hex_preview(buf: bytes, limit: int = 64) -> None:
    if not isinstance(buf, (bytes, bytearray)):
        print(f"(hex preview) not-bytes: {type(buf)}")
        return
    s = buf[:limit].hex()
    if len(buf) > limit:
        s += "..."
    print(f"[HEX {len(buf)}B] {s}")

def _dump(buf: bytes, label: str = "") -> None:
    if label:
        print(label)
    if _hexdump:
        _hexdump(buf)  # pretty hexdump
    else:
        _hex_preview(buf)


# ================== 상수/프레이밍 규약 ==================
CHUNK_DATA_MAX = 63  # frame[1:64]의 유효 데이터 최대치


# ================== CanDataReceiver ==================
class CanDataReceiver:
    """
    CAN-FD 64B 프레임 프레이밍 수신기.
      - frame[0] = hdr
          bit7 = 0 → MIDDLE, bit6..0 = seq(0..127)
          bit7 = 1 → LAST,   bit6..0 = last_len(1..63)
      - frame[1:64] = data(63B; LAST에서는 last_len만 유효, 나머지 0패딩)
    """

    # -----------------------------------------------
    # pcan_manager 요구사항(다음 중 최소 하나 제공):
    #   (A) start_rx(on_frame) / stop_rx(): on_frame(payload_64b: bytes, can_id: int)
    #   (B) ReadFD(channel) -> (result, msg, ts)  (PCANBasic 래핑)
    #       - result == 0 이면 OK
    #       - msg.ID (can_id), msg.DATA (bytes)
    #   (C) read_one() -> (can_id:int, payload_64b:bytes)  또는 (payload_64b, can_id)
    # -----------------------------------------------

    def __init__(self, pcan_manager: Any, filter_can_id: Optional[int] = None):
        self.mgr = pcan_manager
        self.filter_id = filter_can_id

        # 조립기: can_id → (expected_seq, bytearray)
        self._assemblers: Dict[int, Tuple[int, bytearray]] = {}
        self._asm_lock = threading.Lock()

        # 연속 수신 상태
        self._rx_running = False

        # 수신 프레임 큐 (receive_packet 폴링용)
        self._q: Queue[Tuple[bytes, int]] = Queue()

        # 폴링 스레드 핸들 (ReadFD용)
        self._poll_thread: Optional[threading.Thread] = None

    # --------------- 내부 유틸 ---------------
    def _accept(self, can_id: int) -> bool:
        return (self.filter_id is None) or (can_id == self.filter_id)

    def _reset_asm(self, can_id: int, start_seq: int = 0) -> None:
        self._assemblers[can_id] = (start_seq & 0x7F, bytearray())

    def _append_frame(self, can_id: int, payload_64b: bytes) -> Optional[bytes]:
        """프레임 1개 반영. 완료 시 bytes 반환, 아니면 None."""
        if not payload_64b or len(payload_64b) < 64:
            return None

        hdr = payload_64b[0]
        data63 = payload_64b[1:64]
        is_last = bool(hdr & 0x80)
        val = hdr & 0x7F

        with self._asm_lock:
            expected, buf = self._assemblers.get(can_id, (0, bytearray()))

            if is_last:
                last_len = max(0, min(CHUNK_DATA_MAX, val))
                if last_len == 0:
                    # 방어: last_len=0이면 무시
                    return None
                buf.extend(data63[:last_len])
                out = bytes(buf)  # last_len만 추가했으므로 패딩 없음
                self._reset_asm(can_id, 0)
                return out
            else:
                seq = val  # 0..127
                if seq != expected:
                    # 시퀀스 불일치: 재동기화(현재 seq부터 새로 시작)
                    buf = bytearray()
                    expected = seq
                buf.extend(data63[:CHUNK_DATA_MAX])
                expected = (expected + 1) & 0x7F
                self._assemblers[can_id] = (expected, buf)
                return None

    # --------------- 콜백 기반 연속 수신 ---------------
    def start_rx(self, on_packet: Optional[Callable[[bytes, int], None]] = None) -> None:
        """
        이벤트 드리븐 연속 수신 시작.
          - on_packet(data: bytes, can_id: int) : 완성 메시지 콜백 (선택)
          - 동시에 raw 프레임은 내부 큐(self._q)에도 적재되어 receive_packet()에서도 사용 가능.
        """
        if not hasattr(self.mgr, "start_rx"):
            # ReadFD만 있는 경우에는 start_polling() 사용 권장
            self.start_polling()
            return

        self._assemblers.clear()
        self._rx_running = True

        def _on_frame(payload_64b: bytes, can_id: int):
            if not self._rx_running:
                return
            # 필터링
            if not self._accept(can_id):
                return
            # 큐에 raw 프레임 적재 → receive_packet()에서도 사용 가능
            try:
                self._q.put_nowait((payload_64b, can_id))
            except Exception:
                pass
            # 즉시 조립 시도하여 완료되면 on_packet으로 알림
            if on_packet is not None:
                try:
                    done = self._append_frame(can_id, payload_64b)
                    if done is not None:
                        on_packet(done, can_id)
                except Exception:
                    # 콜백에서의 예외는 수신 중단시키지 않음
                    pass

        # pcan_manager.start_rx 시그니처 호환 처리
        try:
            self.mgr.start_rx(_on_frame)
        except TypeError:
            # on_frame=키워드만 받는 매니저도 대응
            self.mgr.start_rx(on_frame=_on_frame)  # type: ignore

    def stop_rx(self) -> None:
        self._rx_running = False
        if hasattr(self.mgr, "stop_rx"):
            try:
                self.mgr.stop_rx()
            except Exception:
                pass

    # --------------- ReadFD 폴링 연속 수신 ---------------
    def start_polling(self, poll_interval_s: float = 0.001) -> None:
        """
        ReadFD 기반 폴링 스레드로 연속 수신. (pcan_manager.start_rx가 없을 때 사용)
        raw 프레임은 큐(self._q)에 적재되고, 조립은 receive_packet() 또는 사용자 로직에서 수행.
        """
        if not (hasattr(self.mgr, "ReadFD") and hasattr(self.mgr, "channel")):
            raise RuntimeError("start_polling: mgr.ReadFD/mgr.channel 미지원")

        self._rx_running = True

        def _th():
            while self._rx_running:
                try:
                    res = self.mgr.ReadFD(self.mgr.channel)
                except Exception:
                    # 드물게 드라이버 에러/리스타트 동안 예외 발생 가능 → 슬립 후 재시도
                    time.sleep(poll_interval_s or 0.001)
                    continue

                if isinstance(res, (list, tuple)) and len(res) == 3:
                    result, msg, _ts = res
                    ok = (getattr(result, "value", result) == 0)  # PCAN_ERROR_OK == 0
                    if not ok:
                        # 큐 비었거나 에러 → 다음 루프
                        if poll_interval_s:
                            time.sleep(poll_interval_s)
                        continue
                    can_id = int(getattr(msg, "ID", -1))
                    data = bytes(getattr(msg, "DATA", b""))
                    if len(data) >= 64 and self._accept(can_id):
                        try:
                            self._q.put_nowait((data[:64], can_id))
                        except Exception:
                            pass
                if poll_interval_s:
                    time.sleep(poll_interval_s)

        self._poll_thread = threading.Thread(target=_th, name="CanRxPoll", daemon=True)
        self._poll_thread.start()

    def stop_polling(self, join_timeout_s: float = 0.5) -> None:
        self._rx_running = False
        th = self._poll_thread
        self._poll_thread = None
        if th and th.is_alive():
            try:
                th.join(timeout=join_timeout_s)
            except Exception:
                pass

    # --------------- 패킷 1개 수신(폴링) ---------------
    def _try_read_one_from_mgr(self) -> Optional[Tuple[bytes, int]]:
        """
        큐가 비었을 때, 매니저에서 직접 프레임 하나 읽기 시도.
        지원되는 API:
          - read_one() -> (can_id, payload64) 또는 (payload64, can_id)
          - ReadFD(channel) -> (res, msg, ts)
        """
        # (1) 사용자 정의 read_one()
        ro = getattr(self.mgr, "read_one", None)
        if callable(ro):
            try:
                item = ro()
                if isinstance(item, tuple) and len(item) >= 2:
                    a, b = item[0], item[1]
                    if isinstance(a, (bytes, bytearray)) and isinstance(b, int):
                        payload, can_id = a, int(b)
                    elif isinstance(b, (bytes, bytearray)) and isinstance(a, int):
                        payload, can_id = b, int(a)
                    else:
                        payload, can_id = None, -1  # type: ignore
                else:
                    payload, can_id = None, -1  # type: ignore
            except Exception:
                payload, can_id = None, -1  # type: ignore

            if payload and len(payload) >= 64 and self._accept(can_id):
                return (bytes(payload[:64]), can_id)

        # (2) PCANBasic ReadFD()
        rf = getattr(self.mgr, "ReadFD", None)
        ch = getattr(self.mgr, "channel", None)
        if callable(rf) and ch is not None:
            try:
                res = rf(ch)
            except Exception:
                res = None
            if isinstance(res, (list, tuple)) and len(res) == 3:
                result, msg, _ts = res
                ok = (getattr(result, "value", result) == 0)
                if ok:
                    can_id = int(getattr(msg, "ID", -1))
                    data = bytes(getattr(msg, "DATA", b""))
                    if len(data) >= 64 and self._accept(can_id):
                        return (data[:64], can_id)

        return None

    def receive_packet(self, timeout_s: Optional[float] = None, verbose: bool = True) -> Optional[bytes]:
        """
        완성된 '메시지'(bytes) 1개를 반환. (없으면 None)
        - 큐에 쌓인 raw 프레임 사용 + 부족하면 mgr에서 직접 1프레임 읽기 시도
        - start_rx/start_polling 중이어도 동작
        - verbose=True 면 프레임/패킷 덤프 출력
        """
        deadline = None if timeout_s is None else (time.time() + float(timeout_s))
        # 임시 조립 버퍼: 단일 메시지 조립용(특정 can_id 모르면 -1 사용)
        tmp_id = -1
        self._reset_asm(tmp_id, 0)

        while True:
            # 타임아웃
            if deadline is not None and time.time() > deadline:
                if verbose:
                    print("Timeout: No complete packet")
                return None

            # (A) 큐에서 1프레임 꺼내기
            try:
                payload, can_id = self._q.get(timeout=0.02)  # 20ms 대기
            except Empty:
                # (B) 큐가 비면 매니저에서 1프레임 직접 읽기
                fetched = self._try_read_one_from_mgr()
                if not fetched:
                    # 과도한 CPU 점유 방지
                    time.sleep(0.001)
                    continue
                payload, can_id = fetched

            # 프레임 덤프(디버그)
            if verbose:
                print(f"[DBG] dequeued frame: id=0x{int(can_id):X}, len={len(payload)}")
                _dump(payload)

            # 조립
            tgt_id = can_id if can_id is not None else tmp_id
            completed = self._append_frame(tgt_id, payload)
            if completed is not None:
                if verbose:
                    print(f"\n[RECEIVED COMPLETE PACKET] from 0x{int(tgt_id):X}, length={len(completed)} bytes")
                    _dump(completed)
                return completed

    # --------------- 호환용 별칭 ---------------
    def receiveData(self, timeout_s: Optional[float] = None) -> Optional[bytes]:
        """레거시 호환: receive_packet() 별칭 (verbose=True)."""
        return self.receive_packet(timeout_s=timeout_s, verbose=True)

    # --------------- 정리 ---------------
    def close(self) -> None:
        """수신 중지 및 리소스 정리."""
        try:
            self.stop_rx()
        except Exception:
            pass
        try:
            self.stop_polling()
        except Exception:
            pass


# ============== 유틸: 프레임 생성(검증용 Mock TX) ==============
def _build_frames_from_bytes(payload: bytes) -> List[bytes]:
    """프레이밍 규약에 맞춰 64B 프레임 목록으로 쪼갭니다."""
    frames: List[bytes] = []
    n = len(payload)
    pos = 0
    seq = 0

    while (n - pos) > CHUNK_DATA_MAX:
        hdr = seq & 0x7F  # MIDDLE
        chunk = payload[pos:pos + CHUNK_DATA_MAX]
        if len(chunk) != CHUNK_DATA_MAX:
            chunk = chunk.ljust(CHUNK_DATA_MAX, b"\x00")
        frames.append(bytes([hdr]) + chunk)
        pos += CHUNK_DATA_MAX
        seq = (seq + 1) & 0x7F

    last_len = n - pos
    if last_len <= 0:
        last_len = 1
        chunk = b"\x00"
    else:
        chunk = payload[pos:pos + last_len]
    pad = b"\x00" * (CHUNK_DATA_MAX - last_len)
    hdr = 0x80 | (last_len & 0x7F)
    frames.append(bytes([hdr]) + chunk + pad)
    return frames

def _build_frames_from_text(text: str, append_lf: bool = True) -> List[bytes]:
    data = text.encode("utf-8", errors="ignore")
    if append_lf:
        data += b"\n"
    return _build_frames_from_bytes(data)


# ============== 검증용 Mock PCAN Manager ==============
class MockPCANManager:
    """
    - start_rx(on_frame): 주기적으로 64B 프레임을 콜백으로 전달
    - read_one(): 큐에서 1프레임 반환 (polling 테스트용)
    """
    def __init__(self, can_id: int = 0xC0, ifg_us: int = 3000, payload_text: str = "Hello from Mock!", append_lf: bool = True):
        self.can_id = int(can_id)
        self.ifg_us = max(0, int(ifg_us))
        self.payload_text = payload_text
        self.append_lf = append_lf

        self._rx_cb: Optional[Callable[[bytes, int], None]] = None
        self._run = False
        self._th: Optional[threading.Thread] = None
        self._q: Queue[Tuple[bytes, int]] = Queue()

        # channel 속성/ReadFD 시뮬레이션과의 호환을 위해 멤버 보유 (실제 사용 안 함)
        self.channel = "MOCK"

    # ---- 모의 송신 스레드 ----
    def _producer(self):
        counter = 0
        while self._run:
            counter += 1
            msg = f"{self.payload_text} #{counter}"
            frames = _build_frames_from_text(msg, append_lf=self.append_lf)
            for f in frames:
                if not self._run:
                    break
                # 큐에도 적재 (read_one용)
                try:
                    self._q.put_nowait((f, self.can_id))
                except Exception:
                    pass
                # 콜백 호출
                cb = self._rx_cb
                if cb:
                    try:
                        cb(f, self.can_id)
                    except Exception:
                        pass
                # 프레임 간 간격
                if self.ifg_us > 0:
                    time.sleep(self.ifg_us / 1_000_000.0)
            # 메시지 간 텀
            time.sleep(0.05)

    def start_rx(self, on_frame: Optional[Callable[[bytes, int], None]] = None, **kwargs):
        self._rx_cb = on_frame
        self._run = True
        self._th = threading.Thread(target=self._producer, name="MockPCAN_RX", daemon=True)
        self._th.start()

    def stop_rx(self):
        self._run = False
        th = self._th
        self._th = None
        if th and th.is_alive():
            try:
                th.join(timeout=0.3)
            except Exception:
                pass

    # polling 테스트를 위한 간단한 read_one()
    def read_one(self):
        try:
            return self._q.get(timeout=0.01)
        except Empty:
            raise Empty()

    # ReadFD 시뮬레이션은 생략 (필요 시 확장 가능)


# ============== main: 검증 실행 ==============
def _parse_can_id(s: str) -> Optional[int]:
    s = (s or "").strip()
    if not s:
        return None
    base = 16 if s.lower().startswith("0x") else 10
    return int(s, base)

def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="CAN-FD 64B framed receiver (test harness)")
    parser.add_argument("--mode", choices=["mock", "pcan"], default="mock", help="수신 소스 선택 (기본: mock)")
    parser.add_argument("--channel", default="PCAN_USBBUS1", help="PCAN channel (pcan 모드일 때)")
    parser.add_argument("--bitrate-fd", default=(
        "f_clock=80000000,nom_brp=2,nom_tseg1=33,nom_tseg2=6,nom_sjw=1,"
        "data_brp=2,data_tseg1=6,data_tseg2=1,data_sjw=1,data_ssp_offset=14"
    ))
    parser.add_argument("--ifg-us", type=int, default=3000, help="프레임 간 간격(us) – mock/pcan 모두 전달 시도")
    parser.add_argument("--filter-id", default="", help="수신 필터 ID(예: 0xC0). 빈칸이면 전체 수신")
    parser.add_argument("--timeout", type=float, default=2.0, help="receive_packet 타임아웃(sec)")
    parser.add_argument("--duration", type=float, default=10.0, help="테스트 총 지속시간(sec). 음수면 무한")
    parser.add_argument("--poll", action="store_true", help="콜백 대신 폴링 모드로 수신")
    parser.add_argument("--text", default="Hello from Mock!", help="mock 모드에서 보낼 텍스트 시작값")
    parser.add_argument("--mock-id", default="0xC0", help="mock 모드의 송신 CAN ID")
    parser.add_argument("--no-lf", action="store_true", help="mock 모드에서 메시지 끝 개행 비활성화")
    parser.add_argument("--quiet", action="store_true", help="프레임/패킷 hexdump 비활성화")

    args = parser.parse_args(argv)

    verbose = not args.quiet
    filt = _parse_can_id(args.filter_id)

    # 매니저 준비
    mgr = None
    if args.mode == "mock":
        mock_id = _parse_can_id(args.mock_id) or 0xC0
        mgr = MockPCANManager(
            can_id=mock_id,
            ifg_us=args.ifg_us,
            payload_text=args.text,
            append_lf=not args.no_lf
        )
    else:
        try:
            from pcan_manager import PCANManager  # type: ignore
        except Exception as e:
            print(f"[ERR] pcan_manager import 실패: {e}", file=sys.stderr)
            return 2
        try:
            mgr = PCANManager()
            # PCANManager가 open(channel, bitrate_fd, ifg_us=...) 를 지원한다는 가정
            try:
                mgr.open(args.channel, args.bitrate_fd, ifg_us=args.ifg_us)  # type: ignore[attr-defined]
            except TypeError:
                mgr.open(args.channel, args.bitrate_fd)  # type: ignore[attr-defined]
            print(f"[INFO] PCAN open OK: channel={args.channel}")
        except Exception as e:
            print(f"[ERR] PCAN open 실패: {e}", file=sys.stderr)
            return 3

    # 리시버 생성
    rx = CanDataReceiver(mgr, filter_can_id=filt)

    # 콜백 핸들러
    def on_packet(data: bytes, can_id: int):
        ts = time.strftime("%H:%M:%S")
        try:
            txt = data.decode("utf-8")
            print(f"[{ts}] [CALLBACK] id=0x{can_id:X}, len={len(data)} → {txt!r}")
        except UnicodeDecodeError:
            hx = binascii.hexlify(data).decode("ascii")
            if len(hx) > 256:
                hx = hx[:256] + "..."
            print(f"[{ts}] [CALLBACK] id=0x{can_id:X}, len={len(data)} → 0x{hx}")

    # 수신 시작
    try:
        if args.poll:
            # 폴링 기반
            if hasattr(mgr, "ReadFD") and hasattr(mgr, "channel"):
                rx.start_polling(poll_interval_s=0.001)
                print("[INFO] RX: start_polling()")
            else:
                # ReadFD가 없으면, mock의 read_one()처럼 receive_packet이 큐 소비만 할 수도 있음
                print("[INFO] RX: polling requested; using receive_packet loop without start_polling")
        else:
            # 콜백 기반
            rx.start_rx(on_packet=on_packet)
            print("[INFO] RX: start_rx()")

        # 검증 루프
        deadline = (time.time() + args.duration) if args.duration >= 0 else None
        if args.poll:
            # receive_packet()으로 직접 완성 메시지 확인
            print("[INFO] enter polling receive loop...")
            while True:
                if deadline is not None and time.time() > deadline:
                    break
                pkt = rx.receive_packet(timeout_s=args.timeout, verbose=verbose)
                if pkt is None:
                    # 타임아웃: 계속
                    continue
                ts = time.strftime("%H:%M:%S")
                try:
                    txt = pkt.decode("utf-8")
                    print(f"[{ts}] [POLL] len={len(pkt)} → {txt!r}")
                except UnicodeDecodeError:
                    hx = binascii.hexlify(pkt).decode("ascii")
                    if len(hx) > 256:
                        hx = hx[:256] + "..."
                    print(f"[{ts}] [POLL] len={len(pkt)} → 0x{hx}")
        else:
            # 콜백 테스트: 지정 시간만큼 대기
            print("[INFO] waiting... (callback mode)")
            while True:
                if deadline is not None and time.time() > deadline:
                    break
                time.sleep(0.1)

    except KeyboardInterrupt:
        print("\n[INFO] Interrupted by user.")
    finally:
        try:
            rx.close()
        except Exception:
            pass
        # pcan 모드에서 mgr에 close가 있으면 닫기
        if args.mode == "pcan":
            try:
                if hasattr(mgr, "close"):
                    mgr.close()  # type: ignore[attr-defined]
            except Exception:
                pass
        print("[INFO] Bye.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
