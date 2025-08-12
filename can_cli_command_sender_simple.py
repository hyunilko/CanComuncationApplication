# can_cli_command_sender_simple.py

import argparse
from PCANBasic import *
import sys
import time
import os


class CanCliCommandSender:
    def __init__(self, channel=PCAN_USBBUS1,
                 bitrate_fd="f_clock=80000000,nom_brp=2,nom_tseg1=33,nom_tseg2=6,nom_sjw=1,data_brp=2,data_tseg1=6,data_tseg2=1,data_sjw=1,data_ssp_offset=14"):
        self.pcan = PCANBasic()
        self.channel = channel
        self.bitrate_fd = TPCANBitrateFD(bitrate_fd.encode('utf-8'))
        self.initialize_pcan()

    def initialize_pcan(self):
        """Initialize PCAN-USB FD channel"""
        result = self.pcan.InitializeFD(self.channel, self.bitrate_fd)
        if result != PCAN_ERROR_OK:
            error_text = self.pcan.GetErrorText(result)[1].decode('utf-8')
            raise Exception(f"Failed to initialize PCAN channel: {error_text}")
        print("PCAN-USB FD initialized successfully")

    def string_to_can_data(self, command):
        """Convert ASCII command string to CAN FD data bytes"""
        command_bytes = command.encode('utf-8')
        if len(command_bytes) > 64:
            raise ValueError(f"Command length ({len(command_bytes)} bytes) exceeds 64 bytes")

        # data = (c_ubyte * 64)()
        data = (c_ubyte * 64).from_buffer_copy(b'0' * 64)
        for i, byte in enumerate(command_bytes):
            data[i] = byte

        # Calculate DLC based on data length
        dlc_map = {0: 0, 1: 1, 2: 2, 3: 3, 4: 4, 5: 5, 6: 6, 7: 7, 8: 8,
                   12: 9, 16: 10, 20: 11, 24: 12, 32: 13, 48: 14, 64: 15}
        length = len(command_bytes)
        for key in sorted(dlc_map.keys()):
            if length <= key:
                return data, dlc_map[key]
        return data, 15  # Default to max DLC for lengths <= 64

    def send_command(self, command):
        """Send a single command as a CAN FD message"""
        try:
            msg = TPCANMsgFD()
            msg.ID = 0xC0  # Standard ID
            msg.MSGTYPE = TPCANMessageType(PCAN_MESSAGE_FD.value | PCAN_MESSAGE_STANDARD.value)
            msg.DATA, msg.DLC = self.string_to_can_data(command)

            result = self.pcan.WriteFD(self.channel, msg)
            if result != PCAN_ERROR_OK:
                error_text = self.pcan.GetErrorText(result)[1].decode('utf-8')
                print(f"Failed to send command '{command}': {error_text}")
                return False
            print(f"Sent command: {command}")
            return True

        except Exception as e:
            print(f"Error sending command '{command}': {str(e)}")
            return False

    def send_config_sequence(self, commands):
        """Send a sequence of configuration commands"""
        for i, cmd in enumerate(commands, 1):
            print(f"[{i}/{len(commands)}] Sending: {cmd}")
            if self.send_command(cmd):
                print("Command sent successfully")
            else:
                print("Command sending failed")
            time.sleep(0.1)  # Delay to prevent bus overload

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

    def close(self):
        """Uninitialize PCAN channel"""
        self.pcan.Uninitialize(self.channel)
        print("PCAN channel uninitialized")


def load_commands_from_file(file_path):
    """Load commands from a text file, ignoring lines starting with '#' or '%'."""
    if not os.path.exists(file_path):
        raise FileNotFoundError(f"Configuration file not found: {file_path}")

    with open(file_path, 'r') as f:
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
        "sensorStart 0 0 0 0"
    ]

    parser = argparse.ArgumentParser(description="Radar Configuration Tool for AWRL6844EVM via PCAN-USB FD")
    parser.add_argument("--command", help="Single command to send")
    parser.add_argument("--all", action="store_true", help="Send all predefined configuration commands")
    parser.add_argument("--file", help="Path to a file containing configuration commands")
    parser.add_argument("--interactive", action="store_true", help="Run in interactive command prompt mode")
    args = parser.parse_args()

    try:
        sender = CanCliCommandSender()

        if args.interactive or not (args.command or args.file or args.all):
            sender.interactive_mode()
        elif args.command:
            sender.send_command(args.command)
        elif args.file:
            commands = load_commands_from_file(args.file)
            sender.send_config_sequence(commands)
        elif args.all:
            sender.send_config_sequence(default_configs)

        sender.close()

    except Exception as e:
        print(f"Error: {str(e)}")
        if 'sender' in locals():
            sender.close()
        sys.exit(1)


if __name__ == "__main__":
    main()