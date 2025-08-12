# pcan_manager.py

#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# pcan_manager.py (Class-based)
# - PCANBasic 초기화/해제
# - 64B FD 프레임 생성 및 전송 (재시도 + 인터프레임 갭)
# - 공개 API:
#     PCANManager.open(channel, bitrate_fd, is_std=True, use_brs=True, ifg_us=1500)
#     PCANManager.close()
#     PCANManager.send_bytes(can_id: int, data: bytes)  # ≤64B 자동 패딩
#     (alias) send_frame, write

from __future__ import annotations
from typing import Optional
import time
from ctypes import c_ubyte
from PCANBasic import *  # noqa
import threading
from queue import Queue, Empty

# --------- helpers ---------
def _as_int_flag(x):
    """PCAN 상수( Int / IntEnum / bytes / bytearray / ctypes.c_ubyte )를 int로 정규화."""
    if isinstance(x, (bytes, bytearray)):
        return x[0]
    v = getattr(x, "value", None)
    if v is not None:
        return _as_int_flag(v)
    return int(x)


def _make_fd_msg(can_id: int, data_64b: bytes, is_std: bool = True, use_brs: bool = True) -> TPCANMsgFD:
    assert len(data_64b) == 64, "data_64b must be exactly 64 bytes"
    msg = TPCANMsgFD()
    msg.ID = can_id
    msg.DLC = 15  # FD DLC for 64 bytes

    flags = _as_int_flag(PCAN_MESSAGE_FD)
    flags |= _as_int_flag(PCAN_MESSAGE_STANDARD) if is_std else _as_int_flag(PCAN_MESSAGE_EXTENDED)
    if use_brs:
        flags |= _as_int_flag(PCAN_MESSAGE_BRS)

    try:
        msg.MSGTYPE = TPCANMessageType(flags)
    except Exception:
        msg.MSGTYPE = flags

    try:
        msg.DATA = (c_ubyte * 64)(*data_64b)
    except Exception:
        # 일부 환경에서 위 라인이 실패할 경우 대비
        for i in range(64):
            msg.DATA[i] = data_64b[i]
    return msg


def _write_fd_blocking(pcan: PCANBasic, channel, msg: TPCANMsgFD, ifg_us: int = 1500, max_retry: int = 100) -> None:
    """PCAN_WriteFD with 재시도(XMTFULL) + 인터프레임 갭 보장."""
    for _ in range(max_retry):
        res = pcan.WriteFD(channel, msg)
        if res == PCAN_ERROR_OK:
            if ifg_us > 0:
                time.sleep(ifg_us / 1_000_000.0)
            return
        if res in (PCAN_ERROR_XMTFULL, PCAN_ERROR_QXMTFULL):
            time.sleep(0.001)  # 1ms backoff
            continue
        raise RuntimeError(f"PCAN WriteFD failed: 0x{int(res):X}")
    raise RuntimeError("PCAN WriteFD: transmit queue full (retries exceeded)")


class PCANManager:
    """PCAN FD 송신 매니저"""

    def __init__(self):
        self._pcan: Optional[PCANBasic] = None
        self._channel = None
        self._is_open = False
        self._is_std = True
        self._use_brs = True
        self._ifg_us = 1500  # 인터프레임 갭 기본값
        self._rx_thread: Optional[threading.Thread] = None
        self._rx_stop = threading.Event()
        self._on_frame = None                # type: Optional[callable]
        self._rx_q: "Queue[bytes]" = Queue() # (옵션) pull 방식도 가능하도록 준비

    @staticmethod
    def _resolve_channel(name_or_val):
        """'PCAN_USBBUS1' 같은 문자열을 상수로 매핑. 숫자나 상수면 그대로 사용."""
        mapping = {
            "PCAN_USBBUS1": PCAN_USBBUS1,
            "PCAN_USBBUS2": PCAN_USBBUS2,
            "PCAN_USBBUS3": PCAN_USBBUS3,
            "PCAN_USBBUS4": PCAN_USBBUS4,
        }
        if isinstance(name_or_val, str):
            return mapping.get(name_or_val, PCAN_USBBUS1)
        return name_or_val

    def open(self,
             channel="PCAN_USBBUS1",
             bitrate_fd=("f_clock=80000000,nom_brp=2,nom_tseg1=33,nom_tseg2=6,nom_sjw=1,"
                         "data_brp=2,data_tseg1=6,data_tseg2=1,data_sjw=1,data_ssp_offset=14"),
             *,
             is_std: bool = True,
             use_brs: bool = True,
             ifg_us: int = 1500):
        """PCAN FD 링크 초기화."""
        if self._is_open:
            return

        self._pcan = PCANBasic()
        self._channel = self._resolve_channel(channel)
        self._is_std = is_std
        self._use_brs = use_brs
        self._ifg_us = int(ifg_us)

        sts = self._pcan.InitializeFD(self._channel, TPCANBitrateFD(str(bitrate_fd).encode("utf-8")))
        if sts != PCAN_ERROR_OK:
            raise RuntimeError(f"PCAN InitializeFD 실패: 0x{int(sts):X}")

        self._is_open = True

    # 호환 이름
    connect = open

    def close(self):
        if not self._is_open or not self._pcan:
            self._is_open = False
            return
        try:
            self._pcan.Uninitialize(self._channel)
        finally:
            self._is_open = False
            self._pcan = None
            self._channel = None

    # 호환 이름
    disconnect = close

    def send_bytes(self, can_id: int, data: bytes):
        """바이트 페이로드 전송(≤64B 자동 패딩)."""
        if not self._is_open or not self._pcan:
            raise RuntimeError("PCANManager가 열려 있지 않습니다. open() 먼저 호출하세요.")

        if not isinstance(data, (bytes, bytearray)):
            raise TypeError("data must be bytes-like")

        if len(data) > 64:
            raise ValueError(f"payload length {len(data)} > 64 (분할 전송 필요)")

        payload = (bytes(data) + b"\x00" * (64 - len(data)))[:64]
        msg = _make_fd_msg(can_id, payload, is_std=self._is_std, use_brs=self._use_brs)
        _write_fd_blocking(self._pcan, self._channel, msg, ifg_us=self._ifg_us)

    # 편의 alias
    def send_frame(self, can_id: int, data: bytes): self.send_bytes(can_id, data)
    def write(self, can_id: int, data: bytes): self.send_bytes(can_id, data)

    def _read_once(self) -> Optional[TPCANMsgFD]:
        """단발 ReadFD. 수신 없으면 None."""
        if not self._is_open or not self._pcan:
            return None
        # ReadFD는 튜플 (result, msg, timestamp) 를 반환
        res, msg, ts = self._pcan.ReadFD(self._channel)
        if res == PCAN_ERROR_QRCVEMPTY:
            return None
        if res != PCAN_ERROR_OK:
            raise RuntimeError(f"PCAN ReadFD failed: 0x{int(res):X}")
        return msg

    @staticmethod
    def _payload_from_msgfd(msg: TPCANMsgFD) -> bytes:
        """64바이트 데이터 영역을 bytes로 추출."""
        try:
            return bytes(bytearray(msg.DATA)[:64])
        except Exception:
            # 일부 바인딩에서는 msg.DATA 가 ctypes 배열
            return bytes((msg.DATA[i] for i in range(64)))

    def start_rx(self, on_frame: callable = None):
        """
        수신 스레드 시작.
        on_frame(payload_bytes: bytes, can_id: int) 콜백을 호출합니다.
        (payload는 64B 원본; 필요 시 GUI에서 ASCII 디코딩/trim)
        """
        if self._rx_thread and self._rx_thread.is_alive():
            return
        self._on_frame = on_frame
        self._rx_stop.clear()

        def _loop():
            while not self._rx_stop.is_set():
                try:
                    msg = self._read_once()
                    if msg is None:
                        time.sleep(PCAN_READ_TIMEOUT_MS / 1000.0)
                        continue
                    payload = self._payload_from_msgfd(msg)
                    self._rx_q.put(payload)
                    if self._on_frame:
                        try:
                            self._on_frame(payload, int(msg.ID))
                        except Exception:
                            pass
                except Exception:
                    # 안정성 위해 과한 예외 전파는 억제하고 잠깐 쉼
                    time.sleep(0.01)

        self._rx_thread = threading.Thread