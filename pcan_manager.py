# pcan_manager.py

from PCANBasic import *

class PCANManager:
    def __init__(self, channel=PCAN_USBBUS1,
                 bitrate_fd="f_clock=80000000,nom_brp=2,nom_tseg1=33,nom_tseg2=6,nom_sjw=1,data_brp=2,data_tseg1=6,data_tseg2=1,data_sjw=1,data_ssp_offset=14"):
        self.pcan = PCANBasic()
        self.channel = channel
        self.bitrate_fd = TPCANBitrateFD(bitrate_fd.encode('utf-8'))
        self.init_done = False

    def initialize(self):
        if not self.init_done:
            result = self.pcan.InitializeFD(self.channel, self.bitrate_fd)
            if result != PCAN_ERROR_OK:
                error_text = self.pcan.GetErrorText(result)[1].decode('utf-8')
                raise Exception(f"PCAN Init Error: {error_text}")
            print("PCAN-USB FD initialized (shared)")
            self.init_done = True

    def close(self):
        if self.init_done:
            self.pcan.Uninitialize(self.channel)
            print("PCAN channel uninitialized")
            self.init_done = False
