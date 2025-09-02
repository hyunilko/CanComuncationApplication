#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# can_simple_sender.py  (PCANManager 기반)

"""
- TX는 PCANBasic를 직접 다루지 않고 pcan_manager.PCANManager를 사용합니다.
- GUI 호환을 위해 다음 메서드를 제공합니다:
  - connect(ifg_us=...), open(...), disconnect()/close()
  - send_line(text: str), send_lines(list[str])
  - _require_mgr()  -> RX가 같은 매니저를 재사용하도록
  - 호환 alias: send_command(), send_config_sequence(), stop_repl()

- 프레임 포맷: CAN FD, DLC=64. 텍스트를 UTF-8로 인코딩하여 64B로 전송(자동 패딩).
"""

import argparse
import sys
import time
import os
from typing import Iterable, Optional

# 우리의 전송 백엔드
from pcan_manager import PCANManager


class CanSimpleSender:
    def __init__(
        self,
        channel: str = "PCAN_USBBUS1",
        bitrate_fd: str = (
            "f_clock=80000000,nom_brp=2,nom_tseg1=33,nom_tseg2=6,nom_sjw=1,"
            "data_brp=2,data_tseg1=6,data_tseg2=1,data_sjw=1,data_ssp_offset=14"
        ),
        *,
        can_id_11bit: int = 0xC0,
        ifg_us: int = 3000,
        is_std: bool = True,
        use_brs: bool = True,
        auto_open: bool = True,
    ):
        """
        channel: "PCAN_USBBUS1" 등 문자열(PCANManager가 내부에서 상수 매핑)
        bitrate_fd: PCAN FD 비트레이트 문자열
        can_id_11bit: 전송에 사용할 기본 CAN ID
        ifg_us: 인터프레임 갭(마이크로초) – PCANManager가 보장
        """
        self.mgr = PCANManager()
        self.channel = channel
        self.bitrate_fd = bitrate_fd
        self.can_id_11bit = int(can_id_11bit)
        self._is_std = bool(is_std)
        self._use_brs = bool(use_brs)
        self._ifg_us = int(ifg_us)

        if auto_open:
            self.open(channel=self.channel, bitrate_fd=self.bitrate_fd, ifg_us=self._ifg_us)

    # ---- 연결 제어 (GUI 호환) ----
    def open(
        self,
        channel: Optional[str] = None,
        bitrate_fd: Optional[str] = None,
        *,
        ifg_us: Optional[int] = None,
        is_std: Optional[bool] = None,
        use_brs: Optional[bool] = None,
    ):
        """PCAN 링크를 연다. 이미 열려있으면 무시."""
        ch = channel if channel is not None else self.channel
        br = bitrate_fd if bitrate_fd is not None else self.bitrate_fd
        ig = self._ifg_us if ifg_us is None else int(ifg_us)
        st = self._is_std if is_std is None else bool(is_std)
        brs = self._use_brs if use_brs is None else bool(use_brs)

        self.mgr.open(channel=ch, bitrate_fd=br, is_std=st, use_brs=brs, ifg_us=ig)

    def connect(self, ifg_us: Optional[int] = None):
        """open()과 동일. GUI에서 connect(ifg_us=...)를 호출해도 동작하도록."""
        self.open(ifg_us=self._ifg_us if ifg_us is None else int(ifg_us))

    def disconnect(self):
        """PCAN 링크 닫기."""
        self.mgr.close()

    close = disconnect  # alias

    def stop_repl(self):
        """GUI 호환용 더미 메서드."""
        pass

    def _require_mgr(self):
        """GUI가 RX 매니저로 재사용할 수 있도록 노출."""
        return self.mgr

    # ---- 전송 ----
    def send_line(self, line: str):
        """
        한 줄 전송. UTF-8 인코딩하여 64B FD 프레임으로 전송(자동 패딩).
        64바이트를 초과하는 경우 ValueError 발생.
        """
        if not isinstance(line, str):
            line = str(line)
        data = line.encode("utf-8")
        if len(data) > 64:
            raise ValueError(f"Command length ({len(data)} bytes) exceeds 64 bytes")
        self.mgr.send_bytes(self.can_id_11bit, data)

    def send_lines(self, lines: Iterable[str]):
        for s in lines:
            self.send_line(s)

    # ---- CLI 호환 alias ----
    def send_command(self, command: str) -> bool:
        """기존 CLI 호환용: 성공/예외 없이 True/False 반환."""
        try:
            self.send_line(command)
            return True
        except Exception as e:
            print(f"Failed to send command '{command}': {e}")
            return False

    def send_config_sequence(self, commands: Iterable[str], delay_s: float = 0.1):
        """기존 CLI 호환용: 각 명령 사이에 약간의 딜레이."""
        commands = list(commands)
        for i, cmd in enumerate(commands, 1):
            print(f"[{i}/{len(commands)}] Sending: {cmd}")
            ok = self.send_command(cmd)
            print("Command sent successfully" if ok else "Command sending failed")
            if delay_s > 0:
                time.sleep(delay_s)

    # ---- CLI 인터랙티브 ----
    def interactive_mode(self):
        """Run interactive command prompt mode"""
        print("Entering interactive mode. Type 'exit' to quit.")
        while True:
            try:
                command = input("COMMAND> ").strip()
                if command.lower() == "exit":
                    break
                if command:
                    self.send_command(command)
                else:
                    print("Empty command ignored")
            except KeyboardInterrupt:
                print("\nInteractive mode terminated by user")
                break
            except Exception as e:
                print(f"Error: {str(e)}")


# --------- 파일 유틸 ---------
def load_commands_from_file(file_path):
    """Load commands from a text file, ignoring lines starting with '#' or '%'."""
    if not os.path.exists(file_path):
        raise FileNotFoundError(f"Configuration file not found: {file_path}")

    with open(file_path, 'r', encoding="utf-8") as f:
        commands = [
            line.strip()
            for line in f
            if line.strip() and not line.strip().startswith(('#', '%'))
        ]
    return commands


def main():
    # Default radar configuration commands
    default_configs = [
        "sensorStop 0",
        "channelCfg 153 255 0",
        "chirpComnCfg 8 0 0 256 1 13.1 3",
        "chirpTimingCfg 6 63 0 160 58",
        "adcDataDitherCfg 1",
        "frameCfg 64 0 1358 1 100 0",
        "gpAdcMeasConfig 0 0",
        "guiMonitor 1 1 0 0 0 1",
        "cfarProcCfg 0 2 8 4 3 0 9.0 0",
        "cfarProcCfg 1 2 4 2 2 1 9.0 0",
        "cfarFovCfg 0 0.25 9.0",
        "cfarFovCfg 1 -20.16 20.16",
        "aoaProcCfg 64 64",
        "aoaFovCfg -60 60 -60 60",
        "clutterRemoval 0",
        "factoryCalibCfg 1 0 44 2 0x1ff000",
        "runtimeCalibCfg 1",
        "antGeometryBoard xWRL6844EVM",
        "adcDataSource 0 adc_test_data_0001.bin",
        "adcLogging 0",
        "lowPowerCfg 1",
        "sensorStart 0 0 0 0",
    ]

    parser = argparse.ArgumentParser(description="Radar Configuration Tool for AWRL6844EVM via PCAN-USB FD")
    parser.add_argument("--command", help="Single command to send")
    parser.add_argument("--all", action="store_true", help="Send all predefined configuration commands")
    parser.add_argument("--file", help="Path to a file containing configuration commands")
    parser.add_argument("--channel", default="PCAN_USBBUS1", help="PCAN channel (e.g., PCAN_USBBUS1)")
    parser.add_argument("--bitrate-fd", default=(
        "f_clock=80000000,nom_brp=2,nom_tseg1=33,nom_tseg2=6,nom_sjw=1,"
        "data_brp=2,data_tseg1=6,data_tseg2=1,data_sjw=1,data_ssp_offset=14"
    ), help="PCAN FD bitrate string")
    parser.add_argument("--ifg-us", type=int, default=3000, help="Inter-frame gap in microseconds")
    parser.add_argument("--id", type=lambda x: int(x, 0), default=0xC0, help="11-bit CAN ID (e.g., 0xC0)")
    args = parser.parse_args()

    try:
        sender = CanSimpleSender(
            channel=args.channel,
            bitrate_fd=args.bitrate_fd,
            can_id_11bit=args.id,
            ifg_us=args.ifg_us,
            auto_open=True,
        )

        if args.command:
            sender.send_command(args.command)
        elif args.file:
            commands = load_commands_from_file(args.file)
            sender.send_config_sequence(commands)
        elif args.all:
            sender.send_config_sequence(default_configs)
        else:
            sender.interactive_mode()

        sender.disconnect()

    except Exception as e:
        print(f"Error: {str(e)}")
        try:
            sender.disconnect()
        except Exception:
            pass
        sys.exit(1)


if __name__ == "__main__":
    main()
