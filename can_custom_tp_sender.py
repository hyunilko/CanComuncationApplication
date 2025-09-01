#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
can_custom_tp_sender.py  (class-based)

표준 API (GUI/스크립트에서 호출):
    CanCustomTpSender.connect(channel: str, bitrate_fd: str) -> None
    CanCustomTpSender.disconnect() -> None
    CanCustomTpSender.send_line(text: str) -> None
    CanCustomTpSender.send_lines(lines: list[str]) -> None
    CanCustomTpSender.start_repl(on_rx: Callable[[str, int], None]) -> None
    CanCustomTpSender.stop_repl() -> None

프레이밍 (64B 고정, FD DLC=15):
  - frame[0] = HDR (1B)
      * bit7 = 0 → MIDDLE 프레임, bit6..0 = SEQ (0..127)
      * bit7 = 1 → LAST   프레임, bit6..0 = last_len (1..63)
  - frame[1:64] = DATA 63B (LAST에서는 last_len 만큼 유효, 나머지 0 패딩)

Application PDU (= APP_PDU):
  - MsgID(1B) + Payload
  - 본 파일은 기존 동작을 유지하면서, DATA 스트림의 첫 바이트로 MsgID(1B)를 **반드시** 삽입합니다.
    (즉, APP_PDU 전체를 63B 단위로 분할하여 전송)

송신:
  텍스트(ASCII, 옵션으로 '\\n' 추가)에 대해 APP_PDU = [MsgID(1B)] + [Payload] 를 만들고
  이를 63바이트씩 절단하여 MIDDLE 프레임(SEQ 0,1,2,...)과 마지막 프레임(LAST + last_len)으로 전송.

수신(REPL):
  위 포맷대로 조립하여 **APP_PDU**를 복원한 뒤, (text, app_msg_id) 형태로 콜백 호출.
  콜백의 두 번째 인자는 "CAN ID"가 아니라 "Application MsgID(1B)" 입니다.

주의:
  총 길이가 정확히 63의 배수여도 마지막 프레임은 반드시 LAST(63)로 마무리합니다.
"""

from __future__ import annotations
from typing import List, Optional, Callable, Dict, Tuple
import argparse
import sys
import os
import time

# ========= 기본 정책값(필요 시 변경) =========
DEFAULT_CHANNEL: str = "PCAN_USBBUS1"
DEFAULT_BITRATE_FD: str = (
    "f_clock=80000000,"
    "nom_brp=2,nom_tseg1=33,nom_tseg2=6,nom_sjw=1,"
    "data_brp=2,data_tseg1=6,data_tseg2=1,data_sjw=1,"
    "data_ssp_offset=14"
)
DEFAULT_CAN_ID_11BIT: int = 0xC0        # Link Layer CAN ID
DEFAULT_COMMAND_MSG_ID: int = 0xFA          # Application PDU MsgID(1B)
DEFAULT_APPEND_LF: bool = True
CHUNK_DATA_MAX: int = 63

DEFAULT_FRAME_GAP_US: int = 3000   # 프레임 간 간격(us) – pcan_manager로 전달
DEFAULT_CMD_GAP_MS: int = 20       # 한 “명령” 전송 후 대기(ms)
# ==========================================

# ===== pcan_manager 백엔드 =====
try:
    from pcan_manager import PCANManager
except Exception as _PM_EXC:
    PCANManager = None  # type: ignore
    _PM_IMPORT_ERROR = _PM_EXC
else:
    _PM_IMPORT_ERROR = None


class CanCustomTpSender:
    """PCAN 기반 CAN-FD(64B) CLI 송수신기 (프레이밍+APP_PDU 삽입)."""

    def __init__(
        self,
        channel: str = DEFAULT_CHANNEL,
        bitrate_fd: str = DEFAULT_BITRATE_FD,
        can_id_11bit: int = DEFAULT_CAN_ID_11BIT,
        app_msg_id: int = DEFAULT_COMMAND_MSG_ID,
        append_lf: bool = DEFAULT_APPEND_LF,
        frame_gap_us: int = DEFAULT_FRAME_GAP_US,
        cmd_gap_ms: int = DEFAULT_CMD_GAP_MS,
    ):
        # 설정값
        self.channel = channel
        self.bitrate_fd = bitrate_fd
        self.can_id_11bit = can_id_11bit
        self.app_msg_id = app_msg_id & 0xFF
        self.append_lf = append_lf
        self.frame_gap_us = max(0, int(frame_gap_us))
        self.cmd_gap_ms = max(0, int(cmd_gap_ms))

        # 런타임 상태
        self._mgr: Optional["PCANManager"] = None
        # on_rx(text, app_msg_id) 로 호출
        self._on_rx_cb: Optional[Callable[[str, int], None]] = None

        # can_id → (expected_seq, bytearray_buffer)
        self._rx_assemblers: Dict[int, Tuple[int, bytearray]] = {}

    # ========================= 연결/해제 =========================
    def connect(self, channel: Optional[str] = None,
                bitrate_fd: Optional[str] = None,
                ifg_us: Optional[int] = None) -> None:
        """PCAN 연결(없으면 생성), 프레임 간격(ifg_us) pcan_manager로 전달."""
        if PCANManager is None:
            raise ImportError(f"pcan_manager import 실패: {_PM_IMPORT_ERROR}")

        if channel is not None:
            self.channel = channel
        if bitrate_fd is not None:
            self.bitrate_fd = bitrate_fd
        if ifg_us is not None:
            self.frame_gap_us = max(0, int(ifg_us))

        if self._mgr is None:
            self._mgr = PCANManager()

        # pcan_manager API 호환 처리
        if hasattr(self._mgr, "open"):
            self._mgr.open(self.channel, self.bitrate_fd, ifg_us=self.frame_gap_us)      # type: ignore[attr-defined]
        elif hasattr(self._mgr, "connect"):
            self._mgr.connect(self.channel, self.bitrate_fd, ifg_us=self.frame_gap_us)   # type: ignore[attr-defined]
        else:
            raise RuntimeError("PCANManager에 open/connect 메서드가 없습니다.")

    def disconnect(self) -> None:
        """PCAN 링크 해제 및 REPL 종료."""
        try:
            self.stop_repl()
        except Exception:
            pass

        if self._mgr is None:
            return

        if hasattr(self._mgr, "close"):
            try:
                self._mgr.close()  # type: ignore[attr-defined]
            except Exception:
                pass
        elif hasattr(self._mgr, "disconnect"):
            try:
                self._mgr.disconnect()  # type: ignore[attr-defined]
            except Exception:
                pass
        self._mgr = None

    def _require_mgr(self) -> "PCANManager":
        if self._mgr is None:
            self.connect(self.channel, self.bitrate_fd, ifg_us=self.frame_gap_us)
        assert self._mgr is not None
        return self._mgr

    # ========================= 송신 =========================
    def _build_frames_from_app_pdu(self, pdu: bytes) -> List[bytes]:
        """
        APP_PDU(=MsgID 1B + Payload)를 63B 단위로 프레이밍하여
        64B(=HDR 1B + DATA 63B) 프레임 배열로 반환.

        - MIDDLE: HDR = SEQ(0..127), DATA = 63B (가득 채움)
        - LAST  : HDR = 0x80 | last_len(1..63), DATA = last_len + padding
        - 전체 길이가 63의 배수일 때도 마지막 프레임은 LAST(63)를 사용
        """
        frames: List[bytes] = []
        n = len(pdu)
        if n <= 0:
            # 빈 APP_PDU는 허용하지 않지만, 안전하게 최소 LAST(1) 프레임을 보냄(데이터 0x00)
            hdr = 0x80 | 0x01
            frames.append(bytes([hdr]) + b"\x00" * CHUNK_DATA_MAX)
            return frames

        full = n // CHUNK_DATA_MAX
        rem = n % CHUNK_DATA_MAX

        seq = 0

        # (1) MIDDLE - 첫 (full-1)개 청크
        for i in range(max(0, full - 1)):
            start = i * CHUNK_DATA_MAX
            end = start + CHUNK_DATA_MAX
            chunk = pdu[start:end]
            if len(chunk) != CHUNK_DATA_MAX:
                chunk = chunk.ljust(CHUNK_DATA_MAX, b"\x00")
            hdr = seq & 0x7F  # bit7=0
            frames.append(bytes([hdr]) + chunk)
            seq = (seq + 1) & 0x7F

        # (2) LAST 프레임
        if rem == 0:
            # 마지막 63B를 LAST(63)로 보냄
            start = (full - 1) * CHUNK_DATA_MAX if full > 0 else 0
            end = start + CHUNK_DATA_MAX
            last_chunk = pdu[start:end]
            if len(last_chunk) != CHUNK_DATA_MAX:
                last_chunk = last_chunk.ljust(CHUNK_DATA_MAX, b"\x00")
            hdr = 0x80 | 0x3F  # 63
            frames.append(bytes([hdr]) + last_chunk)
        else:
            # 남은 rem 바이트를 LAST(rem)로 보냄
            start = (full) * CHUNK_DATA_MAX
            end = start + rem
            last_chunk = pdu[start:end]
            pad = b"\x00" * (CHUNK_DATA_MAX - rem)
            hdr = 0x80 | (rem & 0x7F)
            frames.append(bytes([hdr]) + last_chunk + pad)

        return frames

    def _build_framed_payloads_from_text(self, text: str) -> List[bytes]:
        """
        텍스트를 APP_PDU로 변환 후( MsgID(1B) + Payload ), 64B 프레임 배열로 분할.
        """
        s = text.strip()
        payload = s.encode("ascii", errors="ignore")
        if self.append_lf:
            payload += b"\\n"

        # APP_PDU = [MsgID(1B)] + [Payload]
        pdu = bytes([self.app_msg_id & 0xFF]) + payload
        return self._build_frames_from_app_pdu(pdu)

    def send_line(self, text: str) -> None:
        """CLI 한 줄을 다중 프레임으로 전송(프레이밍+APP_PDU 포함)."""
        if text is None or not text.strip():
            return
        mgr = self._require_mgr()
        frames = self._build_framed_payloads_from_text(text)
        for frame in frames:
            if hasattr(mgr, "send_bytes"):
                mgr.send_bytes(self.can_id_11bit, frame)     # type: ignore[attr-defined]
            elif hasattr(mgr, "send_frame"):
                mgr.send_frame(self.can_id_11bit, frame)     # type: ignore[attr-defined]
            elif hasattr(mgr, "write"):
                mgr.write(self.can_id_11bit, frame)          # type: ignore[attr-defined]
            else:
                raise RuntimeError("PCANManager에 전송 함수(send_bytes/send_frame/write)가 없습니다.")

        if self.cmd_gap_ms > 0:
            time.sleep(self.cmd_gap_ms / 1000.0)

    def send_lines(self, lines: List[str]) -> None:
        """여러 라인을 순차 전송."""
        for line in lines:
            self.send_line(line)

    # ========================= REPL 수신/조립 =========================
    def _assembler_reset(self, can_id: int, start_seq: int = 0) -> None:
        self._rx_assemblers[can_id] = (start_seq & 0x7F, bytearray())

    def _assembler_append(self, can_id: int, seq_or_lastlen: int, is_last: bool, data63: bytes):
        """
        프레임을 조립 버퍼에 반영한다.
        반환값: (completed_text:str, app_msg_id:int) 또는 None
        """
        expected, buf = self._rx_assemblers.get(can_id, (0, bytearray()))
        if is_last:
            last_len = max(0, min(CHUNK_DATA_MAX, seq_or_lastlen))
            buf.extend(data63[:last_len])
            # APP_PDU = [MsgID(1B)] + [Payload...]
            if len(buf) >= 1:
                app_msg_id = buf[0]
                payload = buf[1:]
            else:
                app_msg_id = 0
                payload = b""
            text = payload.rstrip(b"\\x00").decode("ascii", errors="ignore")
            # 완료 후 리셋
            self._assembler_reset(can_id, start_seq=0)
            return (text, app_msg_id)
        else:
            seq = seq_or_lastlen & 0x7F
            if seq != expected:
                # 시퀀스 불일치 → 버퍼 리셋 후 현재 시퀀스 기준으로 재시작
                buf = bytearray()
                expected = seq
            buf.extend(data63[:CHUNK_DATA_MAX])
            expected = (expected + 1) & 0x7F
            self._rx_assemblers[can_id] = (expected, buf)
            return None

    def _on_frame_dispatch(self, payload_64b: bytes, can_id: int):
        """pcan_manager에서 콜백되는 raw 64B 페이로드를 프레이밍 규약에 맞춰 조립."""
        if not payload_64b:
            return
        hdr = payload_64b[0]
        is_last = bool(hdr & 0x80)
        val = hdr & 0x7F
        data63 = payload_64b[1:64]
        res = self._assembler_append(can_id, val, is_last, data63)
        if res and self._on_rx_cb:
            text, app_msg_id = res
            self._on_rx_cb(text, app_msg_id)

    @staticmethod
    def _decode_ascii_from_64b(payload: bytes) -> str:
        """(호환용) 레거시 64B 프레임에서 ASCII 추출: 헤더 무시/데이터 63B만 사용."""
        return payload[1:64].rstrip(b"\\x00").decode("ascii", errors="ignore")

    def start_repl(self, on_rx: Callable[[str, int], None]) -> None:
        """
        수신 루프 시작.
        - pcan_manager.PCANManager.start_rx(on_frame)를 사용
        - on_rx(text, app_msg_id): 완성된 텍스트(조립됨)와 Application MsgID(1B) 전달
        """
        self._on_rx_cb = on_rx
        self._rx_assemblers.clear()
        mgr = self._require_mgr()
        if hasattr(mgr, "start_rx"):
            mgr.start_rx(self._on_frame_dispatch)  # type: ignore[attr-defined]

    def stop_repl(self) -> None:
        mgr = self._mgr
        if mgr is None:
            return
        if hasattr(mgr, "stop_rx"):
            try:
                mgr.stop_rx()  # type: ignore[attr-defined]
            except Exception:
                pass
        self._rx_assemblers.clear()

    # ========================= 편의 유틸 =========================
    @staticmethod
    def load_text_file(path: str) -> List[str]:
        """파일에서 라인 목록을 로드(UTF-8 우선, CP949 폴백). %, 공백 라인 스킵."""
        try:
            with open(path, "r", encoding="utf-8") as f:
                content = f.read()
        except UnicodeDecodeError:
            with open(path, "r", encoding="cp949", errors="ignore") as f:
                content = f.read()

        lines = []
        for s in content.splitlines():
            t = s.rstrip("\\r\\n")
            if not t.strip():
                continue
            if t.lstrip().startswith("%"):
                continue
            lines.append(t)
        return lines

    @staticmethod
    def parse_can_id(x: str) -> int:
        x = x.strip().lower()
        base = 16 if x.startswith("0x") else 10
        v = int(x, base)
        if not (0 <= v < (1 << 11)):
            raise ValueError("CAN 11-bit ID 범위(0..0x7FF) 초과")
        return v

    @staticmethod
    def parse_u8(x: str) -> int:
        x = x.strip().lower()
        base = 16 if x.startswith("0x") else 10
        v = int(x, base) & 0xFF
        return v


# ========================= 메인: 터미널 REPL =========================
def _print_rx_line(text: str, app_msg_id: int):
    sys.stdout.write(f"[MsgID={hex(app_msg_id)}] {text}\\n")
    sys.stdout.flush()


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="CAN CLI command sender (PCAN, 64B FD, framed APP_PDU, REPL)")
    parser.add_argument("--channel", default=DEFAULT_CHANNEL, help="PCAN channel (예: PCAN_USBBUS1)")
    parser.add_argument("--bitrate-fd", default=DEFAULT_BITRATE_FD, help="FD bitrate string")
    parser.add_argument("--id", dest="can_id", default=hex(DEFAULT_CAN_ID_11BIT), help="11-bit CAN ID (예: 0xC0)")
    parser.add_argument("--app-msg-id", default=hex(DEFAULT_COMMAND_MSG_ID),
                        help="Application MsgID (1B) ex) 0x11")
    parser.add_argument("--no-lf", action="store_true", help="라인 끝 개행 '\\n' 추가하지 않음")
    parser.add_argument("--send", nargs="*", default=None, help="인자에 준 라인들을 한번 전송하고 종료")
    parser.add_argument("--load", default=None, help="파일 라인들을 전송하고 종료(%, 공백 라인 스킵)")
    parser.add_argument("--ifg-us", type=int, default=DEFAULT_FRAME_GAP_US,
                        help="프레임 간 간격(마이크로초). pcan_manager로 전달")
    parser.add_argument("--cmd-gap-ms", type=int, default=DEFAULT_CMD_GAP_MS,
                        help="한 명령(여러 프레임) 전송 후 대기(밀리초)")
    args = parser.parse_args(argv)

    # 인스턴스 생성
    try:
        can_id = CanCustomTpSender.parse_can_id(str(args.can_id))
        app_msg_id = CanCustomTpSender.parse_u8(str(args.app_msg_id))
    except Exception as e:
        print(f"[ERR] 인자 파싱 실패: {e}", file=sys.stderr)
        return 2

    sender = CanCustomTpSender(
        channel=args.channel,
        bitrate_fd=args.bitrate_fd,
        can_id_11bit=can_id,
        app_msg_id=app_msg_id,
        append_lf=not args.no_lf,
        frame_gap_us=args.ifg_us,
        cmd_gap_ms=args.cmd_gap_ms,
    )

    # 연결
    try:
        sender.connect()
    except Exception as e:
        print(f"[ERR] Connect 실패: {e}", file=sys.stderr)
        return 1

    # 단발 전송 모드
    if args.send is not None and len(args.send) > 0:
        try:
            sender.send_lines(args.send)
        except Exception as e:
            print(f"[ERR] 전송 실패: {e}", file=sys.stderr)
            sender.disconnect()
            return 1
        sender.disconnect()
        return 0

    if args.load:
        try:
            lines = CanCustomTpSender.load_text_file(args.load)
            sender.send_lines(lines)
        except Exception as e:
            print(f"[ERR] 파일 전송 실패: {e}")
            sender.disconnect()
            return 1
        sender.disconnect()
        return 0

    # 인터랙티브 REPL
    print(f"[INFO] Connected. Channel={sender.channel}, LinkID={hex(sender.can_id_11bit)}, "
          f"AppMsgID={hex(sender.app_msg_id)}, LF={'on' if sender.append_lf else 'off'}")
    print("[INFO] REPL 시작. ':help' 로 도움말, ':quit' 로 종료.")

    sender.start_repl(_print_rx_line)

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
                    "  :help                  - 도움말\n"
                    "  :quit | :exit          - 종료\n"
                    "  :load <file>           - 파일 라인 전송(%, 공백 스킵)\n"
                    "  :id <hex|int>          - 송신 CAN(링크) ID 변경 (예: :id 0xC0)\n"
                    "  :msgid <hex|int>       - Application MsgID(1B) 변경 (예: :msgid 0x11)\n"
                    "  :lf on|off             - 라인 끝 개행('\\n') 추가 on/off\n"
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
                    lines = CanCustomTpSender.load_text_file(path)
                    sender.send_lines(lines)
                    print(f"[INFO] 전송 완료: {len(lines)} lines from {path}")
                except Exception as e:
                    print(f"[ERR] 파일 전송 실패: {e}")
                continue
            if line.startswith(":id "):
                val = line[4:].strip()
                try:
                    sender.can_id_11bit = CanCustomTpSender.parse_can_id(val)
                    print(f"[INFO] CAN(링크) ID 변경: {hex(sender.can_id_11bit)}")
                except Exception as e:
                    print(f"[ERR] 잘못된 ID: {e}")
                continue
            if line.startswith(":msgid "):
                val = line[7:].strip()
                try:
                    sender.app_msg_id = CanCustomTpSender.parse_u8(val)
                    print(f"[INFO] Application MsgID 변경: {hex(sender.app_msg_id)}")
                except Exception as e:
                    print(f"[ERR] 잘못된 MsgID: {e}")
                continue
            if line.startswith(":lf "):
                tok = line[4:].strip().lower()
                if tok in ("on", "1", "true", "yes"):
                    sender.append_lf = True
                elif tok in ("off", "0", "false", "no"):
                    sender.append_lf = False
                else:
                    print("[ERR] :lf on|off 로 지정하세요.")
                    continue
                print(f"[INFO] LF = {'on' if sender.append_lf else 'off'}")
                continue

            # 일반 명령 전송
            try:
                sender.send_line(line)
            except Exception as e:
                print(f"[ERR] 전송 실패: {e}")

    finally:
        sender.stop_repl()
        sender.disconnect()
        print("[INFO] Bye.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
