# pcan_manager.py

from PCANBasic import *
import time
from ctypes import c_ubyte

def _as_int_flag(x):
    """PCAN 상수( Int / IntEnum / bytes / bytearray / ctypes.c_ubyte )를 int로 정규화."""
    # bytes / bytearray → 첫 바이트 값
    if isinstance(x, (bytes, bytearray)):
        return x[0]
    # ctypes.c_ubyte 또는 .value 보유 객체
    v = getattr(x, "value", None)
    if v is not None:
        return _as_int_flag(v)
    try:
        # 일반 정수 캐스팅
        return int(x)
    except Exception as e:
        raise TypeError(f"Cannot convert flag {x!r} to int") from e

def make_fd_msg(can_id: int, data_64b: bytes, is_std: bool = True, use_brs: bool = True) -> TPCANMsgFD:
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
        msg.DATA = bytearray(data_64b)

    return msg

def write_fd_blocking(pcan: PCANBasic, channel, msg: TPCANMsgFD, ifg_us: int = 1500, max_retry: int = 100) -> None:
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
