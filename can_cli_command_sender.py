# can_cli_command_sender.py
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
can_cli_command_sender.py

표준 API (GUI/스크립트에서 호출):
    connect(channel: str, bitrate_fd: str) -> None
    disconnect() -> None
    send_line(text: str) -> None
    send_lines(lines: list[str]) -> None
    start_repl(on_rx: Callable[[str, int], None]) -> None
    stop_repl() -> None

프레이밍 (64B 고정, FD DLC=15):
  - frame[0] = HDR (1B)
      * bit7 = 0 → MIDDLE 프레임, bit6..0 = SEQ (0..127)
      * bit7 = 1 → LAST   프레임, bit6..0 = last_len (1..63)
  - frame[1:64] = DATA 63B (LAST에서는 last_len 만큼 유효, 나머지 0 패딩)

송신:
  텍스트(ASCII, 옵션으로 '\n' 추가)를 63바이트씩 절단하여
  MIDDLE 프레임(SEQ 0,1,2,...)과 마지막 프레임(LAST + last_len)으로 전송.

수신(REPL):
  위 포맷대로 조립하여 완전한 텍스트 한 덩어리마다 콜백/콘솔로 전달.
"""

from __future__ import annotations
from typing import List, Optional, Callable, Dict, Tuple
import argparse
import sys
import os
import time  

# ========= 정책값(필요 시 수정) =========
DEFAULT_CHANNEL: str = "PCAN_USBBUS1"
DEFAULT_BITRATE_FD: str = (
    "f_clock=80000000,"
    "nom_brp=2,nom_tseg1=33,nom_tseg2=6,nom_sjw=1,"
    "data_brp=2,data_tseg1=6,data_tseg2=1,data_sjw=1,"
    "data_ssp_offset=14"
)
CAN_ID_11BIT: int = 0xC0        # 기본 송신 CAN ID (11-bit)
APPEND_LF: bool = True          # 라인 끝에 '\n' 자동 추가
CHUNK_DATA_MAX: int = 63        # 프레임당 데이터 바이트(헤더 제외)

FRAME_GAP_US: int = 3000    # 프레임 사이 간격 (us) – pcan_manager로 전달
CMD_GAP_MS: int = 10        # 한 “명령”을 모두 보낸 뒤 추가 대기 (ms)

# =====================================

# ===== pcan_manager 백엔드 =====
try:
    from pcan_manager import PCANManager
except Exception as _PM_EXC:
    PCANManager = None  # type: ignore
    _PM_IMPORT_ERROR = _PM_EXC
else:
    _PM_IMPORT_ERROR = None

_mgr: Optional["PCANManager"] = None
_on_rx_cb: Optional[Callable[[str, int], None]] = None  # (text, can_id)

# 조립기 상태: can_id → (expected_seq, bytearray_buffer)
_rx_assemblers: Dict[int, Tuple[int, bytearray]] = {}


# ========================= 연결/해제 =========================
def connect(channel: str = DEFAULT_CHANNEL,
            bitrate_fd: str = DEFAULT_BITRATE_FD,
            ifg_us: int = FRAME_GAP_US) -> None:
    global _mgr
    # 1) 모듈 import 확인
    if PCANManager is None:
        raise ImportError(f"pcan_manager import 실패: {_PM_IMPORT_ERROR}")

    # 2) 인스턴스 생성
    if _mgr is None:
        _mgr = PCANManager()

    # 3) open/connect 호출 (pcan_manager는 open에 ifg_us 파라미터 지원)
    if hasattr(_mgr, "open"):
        _mgr.open(channel, bitrate_fd, ifg_us=ifg_us)      # type: ignore[attr-defined]
    elif hasattr(_mgr, "connect"):
        _mgr.connect(channel, bitrate_fd, ifg_us=ifg_us)   # type: ignore[attr-defined]
    else:
        raise RuntimeError("PCANManager에 open/connect 메서드가 없습니다.")


def disconnect() -> None:
    """PCAN 링크 해제 및 REPL 종료."""
    global _mgr
    try:
        stop_repl()
    except Exception:
        pass

    if _mgr is None:
        return
    if hasattr(_mgr, "close"):
        try:
            _mgr.close()                    # type: ignore[attr-defined]
        except Exception:
            pass
    elif hasattr(_mgr, "disconnect"):
        try:
            _mgr.disconnect()               # type: ignore[attr-defined]
        except Exception:
            pass
    _mgr = None


def _require_mgr() -> "PCANManager":
    if _mgr is None:
        connect(DEFAULT_CHANNEL, DEFAULT_BITRATE_FD)
    assert _mgr is not None
    return _mgr


# ========================= 송신 =========================
def _build_framed_payloads_from_text(text: str) -> List[bytes]:
    """
    텍스트를 여러 64B 프레임으로 쪼갠다.
      [0]=HDR(1B), [1:64]=DATA(63B)
      - MIDDLE: HDR = SEQ (0..127), DATA = 63B
      - LAST  : HDR = 0x80 | last_len(1..63), DATA = last_len + padding
    """
    s = text.strip()
    data = s.encode("ascii", errors="ignore")
    if APPEND_LF:
        data += b"\n"

    frames: List[bytes] = []
    if not data:
        return frames

    n = len(data)
    pos = 0
    seq = 0

    # MIDDLE 프레임들
    while (n - pos) > CHUNK_DATA_MAX:
        hdr = seq & 0x7F  # bit7=0
        chunk = data[pos:pos + CHUNK_DATA_MAX]  # 정확히 63B
        if len(chunk) != CHUNK_DATA_MAX:
            # 논리상 발생 X, 방어
            chunk = chunk.ljust(CHUNK_DATA_MAX, b"\x00")
        frame = bytes([hdr]) + chunk  # 1 + 63 = 64
        frames.append(frame)
        pos += CHUNK_DATA_MAX
        seq = (seq + 1) & 0x7F

    # LAST 프레임
    last_len = n - pos
    if last_len <= 0:
        # 이론상 발생 X
        last_len = 1
    if last_len > CHUNK_DATA_MAX:
        # 방어 (상위 루프에서 보장됨)
        last_len = CHUNK_DATA_MAX

    hdr = 0x80 | (last_len & 0x7F)
    last_chunk = data[pos:pos + last_len]
    pad = b"\x00" * (CHUNK_DATA_MAX - last_len)
    frame = bytes([hdr]) + last_chunk + pad
    frames.append(frame)

    return frames


def send_line(text: str) -> None:
    """CLI 한 줄을 다중 프레임으로 전송(프레이밍 적용)."""
    if text is None or not text.strip():
        return
    mgr = _require_mgr()
    frames = _build_framed_payloads_from_text(text)
    for frame in frames:
        if hasattr(mgr, "send_bytes"):
            mgr.send_bytes(CAN_ID_11BIT, frame)     # type: ignore[attr-defined]
        elif hasattr(mgr, "send_frame"):
            mgr.send_frame(CAN_ID_11BIT, frame)     # type: ignore[attr-defined]
        elif hasattr(mgr, "write"):
            mgr.write(CAN_ID_11BIT, frame)          # type: ignore[attr-defined]
        else:
            raise RuntimeError("PCANManager에 전송 함수(send_bytes/send_frame/write)가 없습니다.")

    if CMD_GAP_MS > 0:
        time.sleep(CMD_GAP_MS / 1000.0)


def send_lines(lines: List[str]) -> None:
    """여러 라인을 순차 전송."""
    for line in lines:
        send_line(line)


# ========================= REPL 수신/조립 =========================
def _assembler_reset(can_id: int, start_seq: int = 0) -> None:
    _rx_assemblers[can_id] = (start_seq & 0x7F, bytearray())


def _assembler_append(can_id: int, seq_or_lastlen: int, is_last: bool, data63: bytes):
    """
    프레임을 조립 버퍼에 반영한다.
    반환값: (completed_text or None)
    """
    expected, buf = _rx_assemblers.get(can_id, (0, bytearray()))
    if is_last:
        last_len = max(0, min(CHUNK_DATA_MAX, seq_or_lastlen))
        # 마지막 청크 반영
        buf.extend(data63[:last_len])
        text = buf.rstrip(b"\x00").decode("ascii", errors="ignore")
        # 완료 후 리셋
        _assembler_reset(can_id, start_seq=0)
        return text
    else:
        seq = seq_or_lastlen & 0x7F
        if seq != expected:
            # 시퀀스 불일치 → 버퍼 리셋 후 현재 시퀀스 기준으로 재시작
            buf = bytearray()
            expected = seq
        # 63바이트 전부 추가
        buf.extend(data63[:CHUNK_DATA_MAX])
        expected = (expected + 1) & 0x7F
        _rx_assemblers[can_id] = (expected, buf)
        return None


def _on_frame_dispatch(payload_64b: bytes, can_id: int):
    """pcan_manager에서 콜백되는 raw 64B 페이로드를 프레이밍 규약에 맞춰 조립."""
    if not payload_64b:
        return
    hdr = payload_64b[0]
    is_last = bool(hdr & 0x80)
    val = hdr & 0x7F
    data63 = payload_64b[1:64]
    text = _assembler_append(can_id, val, is_last, data63)
    if text and _on_rx_cb:
        _on_rx_cb(text, can_id)


def _decode_ascii_from_64b(payload: bytes) -> str:
    """(호환용) 레거시 64B 프레임에서 ASCII 추출: 헤더 무시/데이터 63B만 사용."""
    return payload[1:64].rstrip(b"\x00").decode("ascii", errors="ignore")


def start_repl(on_rx: Callable[[str, int], None]) -> None:
    """
    수신 루프 시작.
    - pcan_manager.PCANManager.start_rx(on_frame)를 사용
    - on_rx(text, can_id): 완성된 텍스트(조립됨)와 CAN ID 전달
    """
    global _on_rx_cb
    mgr = _require_mgr()
    _on_rx_cb = on_rx
    # 모든 CAN ID 조립 상태 초기화
    _rx_assemblers.clear()

    if hasattr(mgr, "start_rx"):
        mgr.start_rx(_on_frame_dispatch)     # type: ignore[attr-defined]


def stop_repl() -> None:
    mgr = _mgr
    if mgr is None:
        return
    if hasattr(mgr, "stop_rx"):
        try:
            mgr.stop_rx()                    # type: ignore[attr-defined]
        except Exception:
            pass
    _rx_assemblers.clear()


# ========================= 파일 유틸 =========================
def _load_text_file(path: str) -> List[str]:
    """파일에서 라인 목록을 로드(UTF-8 우선, CP949 폴백). %, 공백 라인 스킵."""
    try:
        with open(path, "r", encoding="utf-8") as f:
            content = f.read()
    except UnicodeDecodeError:
        with open(path, "r", encoding="cp949", errors="ignore") as f:
            content = f.read()

    lines = []
    for s in content.splitlines():
        t = s.rstrip("\r\n")
        if not t.strip():
            continue
        if t.lstrip().startswith("%"):
            continue
        lines.append(t)
    return lines


# ========================= 메인: 터미널 REPL =========================
def _print_rx_line(text: str, can_id: int):
    sys.stdout.write(f"[{hex(can_id)}] {text}\n")
    sys.stdout.flush()


def _parse_can_id(x: str) -> int:
    x = x.strip().lower()
    base = 16 if x.startswith("0x") else 10
    v = int(x, base)
    if not (0 <= v < (1 << 11)):
        raise ValueError("CAN 11-bit ID 범위(0..0x7FF) 초과")
    return v


def main(argv: Optional[List[str]] = None) -> int:
    global CAN_ID_11BIT, APPEND_LF, CMD_GAP_MS, FRAME_GAP_US

    parser = argparse.ArgumentParser(description="CAN CLI command sender (PCAN, 64B FD, framed, REPL)")
    parser.add_argument("--channel", default=DEFAULT_CHANNEL, help="PCAN channel (예: PCAN_USBBUS1)")
    parser.add_argument("--bitrate-fd", default=DEFAULT_BITRATE_FD, help="FD bitrate string")
    parser.add_argument("--id", dest="can_id", default=hex(CAN_ID_11BIT), help="11-bit CAN ID (예: 0xC0)")
    parser.add_argument("--no-lf", action="store_true", help="라인 끝 개행 '\\n' 추가하지 않음")
    parser.add_argument("--send", nargs="*", default=None, help="인자에 준 라인들을 한번 전송하고 종료")
    parser.add_argument("--load", default=None, help="파일 라인들을 전송하고 종료(%, 공백 라인 스킵)")
    parser.add_argument("--ifg-us", type=int, default=FRAME_GAP_US,
                        help="프레임 간 간격(마이크로초). pcan_manager로 전달")
    parser.add_argument("--cmd-gap-ms", type=int, default=CMD_GAP_MS,
                        help="한 명령(여러 프레임) 전송 후 대기(밀리초)")
    args = parser.parse_args(argv)

    FRAME_GAP_US = max(0, int(args.ifg_us))
    CMD_GAP_MS = max(0, int(args.cmd_gap_ms))

    try:
        CAN_ID_11BIT = _parse_can_id(str(args.can_id))
    except Exception as e:
        print(f"[ERR] CAN ID 파싱 실패: {e}", file=sys.stderr)
        return 2
    APPEND_LF = not args.no_lf

    # 연결
    try:
        connect(args.channel, args.bitrate_fd, ifg_us=FRAME_GAP_US)
    except Exception as e:
        print(f"[ERR] Connect 실패: {e}", file=sys.stderr)
        return 1

    # 단발 전송 모드
    if args.send is not None and len(args.send) > 0:
        try:
            send_lines(args.send)
        except Exception as e:
            print(f"[ERR] 전송 실패: {e}", file=sys.stderr)
            disconnect()
            return 1
        disconnect()
        return 0

    if args.load:
        try:
            lines = _load_text_file(args.load)
            send_lines(lines)
        except Exception as e:
            print(f"[ERR] 파일 전송 실패: {e}", file=sys.stderr)
            disconnect()
            return 1
        disconnect()
        return 0

    # 인터랙티브 REPL
    print(f"[INFO] Connected. Channel={args.channel}, ID={hex(CAN_ID_11BIT)}, LF={'on' if APPEND_LF else 'off'}")
    print("[INFO] REPL 시작. ':help' 로 도움말, ':quit' 로 종료.")

    start_repl(_print_rx_line)

    try:
        import readline  # type: ignore
        readline.set_history_length(1000)
    except Exception:
        pass

    try:
        while True:
            try:
                line = input("COMMAND> ").strip()
            except EOFError:
                break
            except KeyboardInterrupt:
                print()
                break

            if not line:
                continue

            # 메타 명령
            if line in (":quit", ":exit"):
                break
            if line == ":help":
                print(
                    "meta commands:\n"
                    "  :help              - 도움말\n"
                    "  :quit | :exit      - 종료\n"
                    "  :load <file>       - 파일 라인 전송(%, 공백 스킵)\n"
                    "  :id <hex|int>      - 송신 CAN ID 변경 (예: :id 0xC0)\n"
                    "  :lf on|off         - 라인 끝 개행('\\n') 추가 on/off\n"
                )
                continue
            if line.startswith(":load "):
                path = line[6:].strip().strip('"').strip("'")
                if not path:
                    print("[ERR] 경로를 지정하세요.")
                    continue
                if not os.path.isfile(path):
                    print(f"[ERR] 파일을 찾을 수 없습니다: {path}")
                    continue
                try:
                    lines = _load_text_file(path)
                    send_lines(lines)
                    print(f"[INFO] 전송 완료: {len(lines)} lines from {path}")
                except Exception as e:
                    print(f"[ERR] 파일 전송 실패: {e}")
                continue
            if line.startswith(":id "):
                val = line[4:].strip()
                try:
                    CAN_ID_11BIT = _parse_can_id(val)
                    print(f"[INFO] CAN ID 변경: {hex(CAN_ID_11BIT)}")
                except Exception as e:
                    print(f"[ERR] 잘못된 ID: {e}")
                continue
            if line.startswith(":lf "):
                tok = line[4:].strip().lower()
                if tok in ("on", "1", "true", "yes"):
                    APPEND_LF = True
                elif tok in ("off", "0", "false", "no"):
                    APPEND_LF = False
                else:
                    print("[ERR] :lf on|off 로 지정하세요.")
                    continue
                print(f"[INFO] LF = {'on' if APPEND_LF else 'off'}")
                continue

            # 일반 명령 전송
            try:
                send_line(line)
            except Exception as e:
                print(f"[ERR] 전송 실패: {e}")

    finally:
        stop_repl()
        disconnect()
        print("[INFO] Bye.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
