# can_cli_command_sender.py
"""
- 초기화 1회 후, COMMAND 프롬프트를 계속 유지하며 반복 전송
- 프레임 포맷: [HDR(1)] + [PAYLOAD(63)] = 64B (FD DLC=15)
  * HDR bit7: 1=LAST, 0=MIDDLE
  * HDR bit6..0: MIDDLE→SEQ(0..127), LAST→last_len(1..63)
- 'exit'/'quit'/'q'/' .exit' 또는 Ctrl+C 로 종료
"""

from __future__ import annotations

import argparse
from PCANBasic import *  # PCAN-Basic Python wrapper (TPCANMsgFD, constants, etc.)
from pcan_manager import make_fd_msg, write_fd_blocking

CHUNK = 63

DEFAULT_BITRATE_FD = (
    "f_clock=80000000,"
    "nom_brp=2,nom_tseg1=33,nom_tseg2=6,nom_sjw=1,"
    "data_brp=2,data_tseg1=6,data_tseg2=1,data_sjw=1,data_ssp_offset=14"
)


class CanCliCommandSender:
    def __init__(self,
                 channel,
                 canid: int,
                 is_std: bool = True,
                 use_brs: bool = True,
                 ifg_us: int = 1500,
                 max_retry: int = 100,
                 bitrate_fd_str: str = DEFAULT_BITRATE_FD):
        self.channel = channel
        self.canid = canid
        self.is_std = is_std
        self.use_brs = use_brs
        self.ifg_us = ifg_us
        self.max_retry = max_retry

        self.pcan = PCANBasic()
        bitrate_fd = TPCANBitrateFD(bitrate_fd_str.encode("ascii"))
        st = self.pcan.InitializeFD(self.channel, bitrate_fd)
        if st != PCAN_ERROR_OK:
            raise RuntimeError(f"InitializeFD failed: 0x{int(st):X}")

    def close(self):
        self.pcan.Uninitialize(PCAN_NONEBUS)

    def send(self, cmd_line: str):
        """한 줄의 CLI 문자열을 64바이트 FD 프레임들로 쪼개 전송"""
        if not cmd_line:
            return
        for frame in self.iter_canfd_frames(cmd_line, CHUNK):
            msg = make_fd_msg(self.canid, frame, is_std=self.is_std, use_brs=self.use_brs)
            write_fd_blocking(self.pcan, self.channel, msg,
                              ifg_us=self.ifg_us, max_retry=self.max_retry)

    @staticmethod
    def iter_canfd_frames(cmd: str, chunk: int = CHUNK):
        """앞에서부터 63Bytes씩 분할. 마지막 프레임은 last_len(1..63)+패딩."""
        payload = cmd.encode("ascii")
        n = len(payload)
        if n == 0:
            return
        pos = 0
        seq = 0
        while (n - pos) > chunk:
            hdr = seq & 0x7F  # MIDDLE
            frame = bytes([hdr]) + payload[pos:pos + chunk]
            assert len(frame) == 1 + chunk == 64
            yield frame
            pos += chunk
            seq = (seq + 1) & 0x7F

        last_len = n - pos
        if last_len <= 0 or last_len > chunk:
            last_len = chunk
        hdr = 0x80 | (last_len & 0x7F)  # LAST
        pad = bytes(chunk - last_len)
        frame = bytes([hdr]) + payload[pos:pos + last_len] + pad
        assert len(frame) == 1 + chunk == 64
        yield frame

    @staticmethod
    def resolve_channel(name: str):
        """문자열 채널명을 PCAN 상수로 변환."""
        try:
            return globals()[name]
        except KeyError:
            return PCAN_USBBUS1


def main():
    ap = argparse.ArgumentParser( # REPL은 Read-Eval-Print Loop의 약자로, 프로그래밍 언어에서 코드를 대화형으로 실행할 수 있는 환경
        description="Send CLI commands over CAN-FD via PCAN (REPL)."
    )
    # 첫 실행 시 보낼 초기 명령(옵션)
    ap.add_argument("cmd", nargs="*", help="Initial CLI command tokens (optional)")
    ap.add_argument("--cmd-str", help="Initial full command as one quoted string")
    ap.add_argument("--from-file", help="Initial command read from a text file")
    # 통신 옵션
    ap.add_argument("--channel", default="PCAN_USBBUS1",
                    help="PCAN channel name (e.g., PCAN_USBBUS1)")
    ap.add_argument("--canid", type=lambda x: int(x, 0), default=0xC0,
                    help="CAN ID (supports 0x..)")
    ap.add_argument("--extended", action="store_true",
                    help="Use 29-bit Extended ID (default: 11-bit Standard)")
    ap.add_argument("--no-brs", action="store_true",
                    help="Disable BRS (data phase at nominal rate)")
    ap.add_argument("--ifg-us", type=int, default=1500,
                    help="Inter-frame gap in microseconds")
    ap.add_argument("--max-retry", type=int, default=100,
                    help="Max retries when TX queue is full")
    ap.add_argument("--bitrate-fd", default=DEFAULT_BITRATE_FD,
                    help="PCAN FD bitrate string")
    args = ap.parse_args()

    channel = CanCliCommandSender.resolve_channel(args.channel)
    is_std = not args.extended
    use_brs = not args.no_brs

    sender = None
    try:
        sender = CanCliCommandSender(channel=channel,
                                     canid=args.canid,
                                     is_std=is_std,
                                     use_brs=use_brs,
                                     ifg_us=args.ifg_us,
                                     max_retry=args.max_retry,
                                     bitrate_fd_str=args.bitrate_fd)

        # 초기 명령 전송(있다면)
        if args.cmd_str:
            init_cmd = args.cmd_str.strip()
        elif args.from_file:
            with open(args.from_file, "r", encoding="utf-8") as f:
                init_cmd = f.read().strip()
        elif args.cmd:
            init_cmd = " ".join(args.cmd).strip()
        else:
            init_cmd = ""

        if init_cmd:
            sender.send(init_cmd)
            print("[OK] Sent (initial)")

        # REPL
        print("Enter CLI commands. Type 'exit'/'quit'/'q' to leave.")
        while True:
            try:
                cmd_line = input("COMMAND> ").strip()
            except KeyboardInterrupt:
                print("\n^C")
                break
            except EOFError:
                print()
                break

            if not cmd_line:
                continue
            if cmd_line.lower() in ("exit", "quit", "q", ".exit"):
                break

            try:
                sender.send(cmd_line)
                print("[OK] Sent")
            except Exception as e:
                print(f"[ERR] {e}")

    finally:
        if sender is not None:
            sender.close()


if __name__ == "__main__":
    main()
