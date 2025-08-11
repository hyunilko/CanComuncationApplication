# can_data_sender.py

import argparse
import time
import os
from ctypes import c_ubyte
from PCANBasic import TPCANMsgFD, PCAN_MESSAGE_FD, PCAN_MESSAGE_STANDARD, TPCANMessageType, PCAN_ERROR_OK
from pcan_manager import PCANManager

CAN_ID = 0xC0
CAN_FD_PAYLOAD = 64

def load_commands_from_file(file_path):
    if not os.path.exists(file_path):
        raise FileNotFoundError(f"Configuration file not found: {file_path}")
    with open(file_path, 'r') as f:
        commands = [
            line.strip()
            for line in f
            if line.strip() and not line.strip().startswith(('#', '%'))
        ]
    return commands

class CanDataSender:
    def __init__(self, pcan_manager: PCANManager):
        self.pcan = pcan_manager.pcan
        self.channel = pcan_manager.channel

    def send_command(self, command: str):
        data = command.encode('utf-8')
        if len(data) > CAN_FD_PAYLOAD:
            raise ValueError(f"Command too long for one CAN FD frame ({len(data)} > {CAN_FD_PAYLOAD})")

        if len(data) < CAN_FD_PAYLOAD:
            padding = b'\x00' * (CAN_FD_PAYLOAD - len(data))
            payload = data + padding
        else:
            payload = data

        msg = TPCANMsgFD()
        msg.ID = CAN_ID
        msg.MSGTYPE = TPCANMessageType(PCAN_MESSAGE_FD.value | PCAN_MESSAGE_STANDARD.value)
        msg.DATA = (c_ubyte * CAN_FD_PAYLOAD).from_buffer_copy(payload)
        msg.DLC = 15  # 64 bytes for CAN FD

        result = self.pcan.WriteFD(self.channel, msg)
        if result != PCAN_ERROR_OK:
            error_text = self.pcan.GetErrorText(result)[1].decode('utf-8')
            print(f"Send error: {error_text}")
            return False

        print(f"Command sent: '{command}' (len={len(data)})")
        print(f"Padding with {'0' * (CAN_FD_PAYLOAD - len(data)) if len(data)<CAN_FD_PAYLOAD else ''}")
        return True

    def send_long_command(self, command: str):
        data = command.encode('utf-8')
        total_len = len(data)
        seq = 0
        pos = 0
        max_payload = 63

        while pos < total_len:
            chunk = data[pos:pos + max_payload]
            is_last = (pos + max_payload >= total_len)  # 중요! 남은게 max_payload 이하일 때만 마지막
            pbf = 1 if is_last else 0

            if not is_last:
                seq_or_len = seq & 0x7F
            else:
                seq_or_len = len(chunk) & 0x7F
                if seq_or_len == 0:
                    # 길이 0 패킷 보내면 안 됨
                    print("SKIP: Not sending empty last frame.")
                    break

            header = ((pbf & 0x01) << 7) | (seq_or_len)
            frame = bytes([header]) + chunk
            frame += b'\x00' * (64 - len(frame))

            msg = TPCANMsgFD()
            msg.ID = CAN_ID
            msg.MSGTYPE = TPCANMessageType(PCAN_MESSAGE_FD.value | PCAN_MESSAGE_STANDARD.value)
            msg.DATA = (c_ubyte * 64).from_buffer_copy(frame)
            msg.DLC = 15

            result = self.pcan.WriteFD(self.channel, msg)
            if result != PCAN_ERROR_OK:
                error_text = self.pcan.GetErrorText(result)[1].decode('utf-8')
                print(f"Send error (seq={seq}): {error_text}")
                return False

            pos += len(chunk)
            seq += 1
            time.sleep(0.01)
        print("Command sent (multi-frame, PACKET_LAST seq_no=actual_data_len)")
        return True

    def send_config_sequence(self, commands):
        for i, cmd in enumerate(commands, 1):
            print(f"[{i}/{len(commands)}] Sending: {cmd}")
            ok = self.send_command(cmd)
            if ok:
                print("Command sent successfully")
            else:
                print("Command sending failed")
            time.sleep(0.1)

    def interactive_mode(self):
        print("Interactive mode (type 'exit' to quit)")
        while True:
            try:
                cmd = input("COMMAND> ").strip()
                if cmd.lower() == "exit": break
                if cmd: self.send_long_command(cmd)
            except KeyboardInterrupt:
                print("\nTerminated"); break

def main():
    parser = argparse.ArgumentParser(description="CanDataSender (CAN FD 64 bytes, '0' padding)")
    parser.add_argument("--command", help="Single command to send")
    parser.add_argument("--file", help="Path to a file containing configuration commands")
    parser.add_argument("--interactive", action="store_true", help="Interactive mode")
    args = parser.parse_args()

    pcan_manager = PCANManager()
    pcan_manager.initialize()
    try:
        sender = CanDataSender(pcan_manager)
        if args.interactive or not (args.command or args.file):
            sender.interactive_mode()
        elif args.command:
            sender.send_command(args.command)
        elif args.file:
            commands = load_commands_from_file(args.file)
            sender.send_config_sequence(commands)
    finally:
        pcan_manager.close()

if __name__ == "__main__":
    main()
