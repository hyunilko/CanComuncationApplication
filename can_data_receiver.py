#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# can_data_receiver.py

import time
from ctypes import c_ubyte
from PCANBasic import PCAN_ERROR_OK, PCAN_ERROR_QRCVEMPTY
from pcan_manager import PCANManager
from hexdump import hexdump

CAN_ID = 0xC0; FRAME_SIZE = 64

class CanDataReceiver:
    def __init__(self, pcan_manager: PCANManager):
        self.pcan = pcan_manager.pcan
        self.channel = pcan_manager.channel

    def receiveData(self, timeout_s=None):
        buf = bytearray(); seq_expect = 0; frame_count = 0
        start_time = time.time()
        while True:
            if (time.time() - start_time) > timeout_s:
                print("Timeout: No complete packet")
                return None

            res_tuple = self.pcan.ReadFD(self.channel)
            if len(res_tuple) != 3:
                print("Unexpected ReadFD return:", res_tuple)
                continue
            result, msg, timestamp = res_tuple

            if result == PCAN_ERROR_QRCVEMPTY:
                time.sleep(0.001); continue
            elif result != PCAN_ERROR_OK:
                err_text = self.pcan.GetErrorText(result)[1].decode('utf-8')
                print(f"CAN Read Error: {err_text}"); continue

            # if msg.ID != CAN_ID: continue

            data = bytes(msg.DATA)
            print(f"Received ID: 0x{msg.ID:X}  DLC: {msg.DLC}  Data: {bytes(msg.DATA[:msg.DLC])}")
            hexdump(data[:msg.DLC])

            header = data[0]
            pbf = (header & 0x80) >> 7
            seq = header & 0x7F
            chunk = data[1:FRAME_SIZE]

            if seq != seq_expect:
                print(f"Seq Error: expect {seq_expect}, got {seq}")
                buf = bytearray(); seq_expect = 0; frame_count = 0
                continue

            buf += chunk; seq_expect += 1; frame_count += 1

            if pbf == 1:
                result_data = buf.rstrip(b'\x00')
                print(f"Received {frame_count} frames, Length: {len(result_data)}")
                return result_data

def main():
    pcan_manager = PCANManager()
    pcan_manager.initialize()
    receiver = CanDataReceiver(pcan_manager)
    try:
        print("Receiving CAN FD data (seq protocol)...")
        while True:
            result = receiver.receiveData(timeout_s=30)
            if result:
                try:
                    text = result.decode('utf-8')
                    print(f"\n[RECEIVED MESSAGE]\n{text}\n")
                except UnicodeDecodeError:
                    print(f"\n[RECEIVED BINARY] {len(result)} bytes\n")
            else:
                print("No complete packet received (timeout).")
    finally:
        pcan_manager.close()

if __name__ == "__main__":
    main()
