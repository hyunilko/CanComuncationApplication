#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
can_isotp_receiver.py

ISO-TP 완성 패킷 수신기 (Application Layer: [Msg ID | Payload...])
 - python-can + (가능시) python-can-isotp 스택으로 FF/CF 조립
 - 스택 조립 불능 시에도 수동 조립기로 FF/CF를 복구하여 APP PDU 로그/저장 보장
 - 수신 패킷을 (payload, msg_id) 형태로 콜백/큐로 전달
 - 파일 저장 포맷(옵션 save_path):
     MSG ID: 0x<hex>\n<payload_hex(대문자, 공백없음)>\n\n

의존:
    pip install python-can can-isotp
"""

from __future__ import annotations
from typing import Optional, Set, Callable, Tuple
import os
import sys
import time
import threading
import queue
import binascii
import re

import can

try:
    import isotp  # python-can-isotp
    _HAS_ISOTP = True
except Exception:
    _HAS_ISOTP = False

# stdout 라인버퍼링(스레드 로그 즉시 표시)
try:
    sys.stdout.reconfigure(line_buffering=True)
except Exception:
    pass

# ========== 유틸/공통 ==========
def _hex_bytes(b: bytes) -> str:
    return " ".join(f"{x:02X}" for x in b)

def _hex_compact(b: bytes) -> str:
    return binascii.hexlify(b).decode("ascii").upper()

def _get_attr(obj, name, default=None):
    return getattr(obj, name, default)

def _env_int(name: str, default: Optional[int]) -> Optional[int]:
    v = os.environ.get(name)
    if v is None:
        return default
    try:
        return int(v, 0)
    except Exception:
        return default

def _env_bool(name: str, default: Optional[bool]) -> Optional[bool]:
    v = os.environ.get(name)
    if v is None:
        return default
    v = v.strip().lower()
    if v in ("1", "true", "yes", "y", "on"): return True
    if v in ("0", "false", "no", "n", "off"): return False
    return default

def _env_float(name: str, default: Optional[float]) -> Optional[float]:
    v = os.environ.get(name)
    if v is None:
        return default
    try:
        return float(v)
    except Exception:
        return default

def _resolve_bus(rx_mgr) -> Optional[can.BusABC]:
    if isinstance(rx_mgr, can.BusABC):
        return rx_mgr
    for name in ("bus", "_bus"):
        b = _get_attr(rx_mgr, name, None)
        if isinstance(b, can.BusABC):
            return b
    for name in ("get_bus", "bus"):
        fn = _get_attr(rx_mgr, name, None)
        if callable(fn):
            try:
                b = fn()
                if isinstance(b, can.BusABC):
                    return b
            except Exception:
                pass
    return None

# ========== Application Layer ==========
def unpack_app(pdu: bytes) -> Tuple[int, bytes]:
    """[MsgID][Payload...] → (msg_id, payload). 비정상/공백이면 (-1, 원문)."""
    if not pdu:
        return (-1, b"")
    return (pdu[0], pdu[1:])

def payload_to_text_or_hex(payload: bytes) -> str:
    # 항상 공백없는 대문자 HEX로 저장
    return _hex_compact(payload)

def _log_app_pdu(msg_id: int, payload: bytes, preview_max: int = 64) -> None:
    """
    조립 완료된 APP PDU 로그(항상 한 줄).
    포맷 예:
      [APP PDU RX] ID=0x64 LEN=193 DATA[:64]=00 00 ...
    LEN = MsgID(1) + PayloadLen
    preview_max는 ENV APP_PREVIEW_MAX(기본 64)로 조절
    """
    try:
        pv_max = int(os.environ.get("APP_PREVIEW_MAX", str(preview_max)))
    except Exception:
        pv_max = preview_max
    pdu_len = 1 + len(payload)
    pv = payload[:pv_max]
    print(
        f"[APP PDU RX] ID=0x{msg_id:02X} LEN={pdu_len} "
        f"DATA[:{len(pv)}]={_hex_bytes(pv)}",
        flush=True, file=sys.stdout
    )

# ========== 보조: SF 판별/추출 (클래식 폴백용) ==========
def _looks_like_isotp_sf(data: bytes) -> bool:
    if len(data) < 2:
        return False
    pci = data[0]
    ptype = (pci >> 4) & 0xF
    if ptype != 0:
        return False
    if len(data) <= 8:
        sf_len = (pci & 0xF)
        return (sf_len > 0) and ((1 + sf_len) <= len(data))
    if (pci & 0xF) != 0 or len(data) < 2:
        return False
    sf_len = data[1]
    return (2 + sf_len) <= len(data)

def _extract_isotp_sf_payload(data: bytes) -> Optional[bytes]:
    if not _looks_like_isotp_sf(data):
        return None
    if len(data) <= 8:
        return data[1:1 + (data[0] & 0xF)]
    return data[2:2 + data[1]]

# ========== Flow Watch (진행 감시) ==========
class _IsoTpFlowWatch(can.Listener):
    def __init__(self, src_can_id: int,
                 first_cf_timeout_s: float,
                 cf_gap_timeout_s: float):
        super().__init__()
        self._src = int(src_can_id)
        self._first_cf_to = float(first_cf_timeout_s)
        self._cf_gap_to = float(cf_gap_timeout_s)
        self._in_rx = False
        self._t_ff = 0.0
        self._t_last_cf = 0.0
        self.reset_needed = False

    @staticmethod
    def _pci_type(data: bytes) -> Optional[int]:
        if not data:
            return None
        return (data[0] >> 4) & 0xF

    def on_message_received(self, msg: "can.Message"):
        try:
            if int(msg.arbitration_id) != self._src:
                return
            data = bytes(msg.data or b"")
            ptype = self._pci_type(data)
            now = time.monotonic()

            if ptype == 1:  # FF
                self._in_rx = True
                self._t_ff = now
                self._t_last_cf = 0.0
            elif ptype == 2:  # CF
                if self._in_rx:
                    self._t_last_cf = now
            elif ptype == 0:  # SF
                self._in_rx = False
                self._t_ff = 0.0
                self._t_last_cf = 0.0

            if self._in_rx:
                if self._t_last_cf == 0.0:
                    if (now - self._t_ff) > self._first_cf_to:
                        self.reset_needed = True
                else:
                    if (now - self._t_last_cf) > self._cf_gap_to:
                        self.reset_needed = True
        except Exception:
            pass

    def clear(self):
        self._in_rx = False
        self._t_ff = 0.0
        self._t_last_cf = 0.0
        self.reset_needed = False

# ========== RAW 탭 리스너 ==========
class _TapAllListener(can.Listener):
    @staticmethod
    def _ptype(data: bytes):
        if not data: return None
        return (data[0] >> 4) & 0xF

    def on_message_received(self, msg: "can.Message"):
        try:
            cid  = int(msg.arbitration_id)
            data = bytes(msg.data or b"")
            ptype = self._ptype(data)
            tag = {0:"SF",1:"FF",2:"CF",3:"FC"}.get(ptype,"??")
            hx_head = " ".join(f"{b:02X}" for b in data[:16])
            print(f"[RAW RX] ID=0x{cid:X} LEN={len(data)} PCI={tag} DATA[:16]={hx_head}", flush=True)
        except Exception:
            pass

# ========== 폴백: 단일 프레임(SF)만 APP 전달 ==========
class _RawListener(can.Listener):
    """스택 비사용/실패 시, SF만 파싱해서 (body,msg_id) 전달."""
    def __init__(self,
                 out_q: "queue.Queue[Tuple[bytes,int]]",
                 emit_app: Callable[[int, bytes], None],
                 filter_ids: Optional[Set[int]],
                 debug_raw: bool = False):
        super().__init__()
        self._q = out_q
        self._emit_app = emit_app
        self._filter = set(filter_ids) if filter_ids else None
        self._debug_raw = debug_raw

    def on_message_received(self, msg: "can.Message"):
        try:
            can_id = int(msg.arbitration_id)
            data = bytes(msg.data or b"")
            if (self._filter is not None) and (can_id not in self._filter):
                return
            payload = _extract_isotp_sf_payload(data)
            if payload is None:
                return
            msg_id, body = unpack_app(payload)
            self._emit_app(msg_id, body)
        except Exception:
            pass

# ========== 수동 조립기: FF/CF 조합(스택 실패 대비) ==========
class _IsoTpProbeAssembler(can.Listener):
    """ID 하나에 대해 FF/CF 조립 (수신만). 라이브러리 스택이 동작하지 않아도 APP PDU를 보장."""
    def __init__(self, target_rxid: int, emit: Callable[[int, bytes], None],
                 first_cf_timeout_s: float = 1.0, cf_gap_timeout_s: float = 0.5):
        super().__init__()
        self._rxid = int(target_rxid)
        self._emit = emit
        self._first_cf_to = float(first_cf_timeout_s)
        self._cf_gap_to = float(cf_gap_timeout_s)
        self._reset()

    def _reset(self):
        self._in_rx = False
        self._need_len = 0
        self._buf = bytearray()
        self._expect_sn = 1
        self._t_start = 0.0
        self._t_last = 0.0

    @staticmethod
    def _ptype(data: bytes) -> int:
        return (data[0] >> 4) & 0xF if data else -1

    def on_message_received(self, msg: "can.Message"):
        try:
            if int(msg.arbitration_id) != self._rxid:
                return
            data = bytes(msg.data or b"")
            if not data:
                return
            ptype = self._ptype(data)
            now = time.monotonic()

            if ptype == 0:  # SF
                if len(data) <= 8:
                    sf_len = data[0] & 0xF
                    pay = data[1:1+sf_len]
                else:
                    if (data[0] & 0xF) != 0 or len(data) < 2:
                        return
                    sf_len = data[1]
                    pay = data[2:2+sf_len]
                if not pay:
                    return
                msg_id = pay[0]
                body = pay[1:]
                self._emit(msg_id, body)
                self._reset()
                return

            if ptype == 1:  # FF
                if len(data) < 2:
                    return
                total = ((data[0] & 0x0F) << 8) | data[1]
                pay_off = 2
                chunk = data[pay_off:]
                self._buf = bytearray(chunk)
                self._need_len = int(total)
                self._in_rx = True
                self._expect_sn = 1
                self._t_start = now
                self._t_last = 0.0
                return

            if ptype == 2 and self._in_rx:  # CF
                sn = data[0] & 0x0F
                if self._t_last == 0.0:
                    if (now - self._t_start) > self._first_cf_to:
                        self._reset()
                        return
                else:
                    if (now - self._t_last) > self._cf_gap_to:
                        self._reset()
                        return
                if sn != self._expect_sn:
                    self._reset()
                    return
                self._expect_sn = (self._expect_sn + 1) & 0x0F
                chunk = data[1:]
                self._buf.extend(chunk)
                self._t_last = now

                if len(self._buf) >= self._need_len:
                    pdu = bytes(self._buf[:self._need_len])
                    if pdu:
                        msg_id = pdu[0]
                        body = pdu[1:]
                        self._emit(msg_id, body)
                    self._reset()
                return

            # ptype == 3 (FC) or 기타: 무시
        except Exception:
            pass

# ========== 공개 수신기 ==========
class CanIsoTpReceiver:
    """
    ISO-TP 완성 패킷 수신기
      - 가능: NotifierBasedCanStack으로 FF/CF까지 조립 후 Application Layer 언패킹
      - 불가: 수동 조립기로 복구
    콜백 시그니처: on_packet(payload: bytes, msg_id: int)
    저장 포맷: "MSG ID: 0x%X\n<PAYLOAD_HEX>\n\n"
    """

    def __init__(self, rx_mgr, filter_can_ids: Optional[Set[int]] = None, save_path: Optional[str] = None):
        self._mgr = rx_mgr
        self._bus: Optional[can.BusABC] = _resolve_bus(rx_mgr)
        if self._bus is None:
            raise RuntimeError("RX 매니저에서 CAN Bus를 얻지 못했습니다.")

        self._txid = _env_int("ISOTP_TXID", _get_attr(rx_mgr, "txid", None))
        self._rxid = _env_int("ISOTP_RXID", _get_attr(rx_mgr, "rxid", None))
        self._extended = _env_bool("ISOTP_EXT", _get_attr(rx_mgr, "extended", None))
        self._fd = _env_bool("ISOTP_FD", _get_attr(rx_mgr, "fd", None))

        self._q: "queue.Queue[Tuple[bytes,int]]" = queue.Queue(maxsize=4096)
        self._on_packet: Optional[Callable[[bytes,int], None]] = None
        self._stop_evt = threading.Event()

        self._use_isotp = _HAS_ISOTP and (self._txid is not None) and (self._rxid is not None)
        self._stack: Optional[isotp.NotifierBasedCanStack] = None
        self._thread: Optional[threading.Thread] = None
        self._notifier: Optional[can.Notifier] = None
        self._own_notifier: bool = False
        self._tap: Optional[_TapAllListener] = None
        self._raw_listener: Optional[_RawListener] = None
        self._manual: Optional[_IsoTpProbeAssembler] = None

        self._filter_ids = set(filter_can_ids) if filter_can_ids else None
        self._flow_watch: Optional[_IsoTpFlowWatch] = None
        self._address = None
        self._params = None

        self._save_path = save_path
        self._raw_tap_enabled = os.environ.get("RAW_TAP", "1").lower() not in ("0","false","no","off")

        # 중복 방지(스택/수동 동시조립 시)
        self._last_app_key = None
        self._last_app_ts = 0.0
        self._dedup_window_s = float(os.environ.get("APP_DEDUP_SEC", "0.25"))

    # 저장
    def _save_record(self, msg_id: int, payload: bytes):
        if not self._save_path:
            return
        try:
            with open(self._save_path, "a", encoding="utf-8", newline="") as f:
                f.write(f"MSG_ID: 0x{int(msg_id):X}\n")
                f.write(payload_to_text_or_hex(payload))
                f.write("\n\n")
        except Exception:
            pass

    # 중복 방지 포함 APP emit
    def _emit_app(self, msg_id: int, body: bytes):
        # dedup key: (id, len, first 8 bytes)
        key = (int(msg_id) & 0xFF, len(body), bytes(body[:8]))
        now = time.monotonic()
        if self._last_app_key == key and (now - self._last_app_ts) < self._dedup_window_s:
            return
        self._last_app_key = key
        self._last_app_ts = now

        _log_app_pdu(msg_id, body)
        try:
            self._q.put_nowait((body, msg_id))
        except queue.Full:
            pass
        self._save_record(msg_id, body)
        if self._on_packet:
            try:
                self._on_packet(body, msg_id)
            except Exception:
                pass

    def set_raw_tap(self, enabled: bool):
        self._raw_tap_enabled = bool(enabled)
        if not self._notifier:
            return
        if not enabled and self._tap is not None:
            try:
                self._notifier.remove_listener(self._tap)
            except Exception:
                pass
            self._tap = None
        elif enabled and self._tap is None:
            self._tap = _TapAllListener()
            try:
                self._notifier.add_listener(self._tap)
            except Exception:
                pass

    def start_rx(self, on_packet: Optional[Callable[[bytes,int], None]] = None):
        self._on_packet = on_packet
        self._stop_evt.clear()

        # Notifier 준비
        shared_notifier = _get_attr(self._mgr, "notifier", None)
        if isinstance(shared_notifier, can.Notifier):
            self._notifier = shared_notifier
            self._own_notifier = False
        else:
            self._notifier = can.Notifier(self._bus, [], timeout=0.05)
            self._own_notifier = True

        # RAW 탭
        if self._raw_tap_enabled and self._tap is None:
            self._tap = _TapAllListener()
            try:
                self._notifier.add_listener(self._tap)
            except Exception:
                pass

        # 수동 조립기(항상 부착: 스택 실패/미동작 대비)
        self._manual = _IsoTpProbeAssembler(int(self._rxid or 0), self._emit_app,
                                            first_cf_timeout_s=float(os.environ.get("APP_FIRST_CF_TO", "1.0")),
                                            cf_gap_timeout_s=float(os.environ.get("APP_CF_GAP_TO", "0.5")))
        try:
            self._notifier.add_listener(self._manual)
        except Exception:
            pass

        if self._use_isotp:
            addr_mode = (isotp.AddressingMode.Normal_29bits
                        if (self._extended is True)
                        else isotp.AddressingMode.Normal_11bits)
            address = isotp.Address(addr_mode, txid=int(self._txid), rxid=int(self._rxid))
            params = dict(
                stmin=0,
                blocksize=0,
                wftmax=3,
                can_fd=bool(self._fd),
                tx_data_length=64 if self._fd else 8,
                tx_data_min_length=64 if self._fd else 8,
                tx_padding=0x00,
                rx_flowcontrol_timeout=int(float(os.environ.get("ISOTP_RX_FC_TO_MS", "3000"))),
                rx_consecutive_frame_timeout=int(float(os.environ.get("ISOTP_RX_CF_TO_MS", "3000"))),
            )
            self._address = address
            self._params = params

            print(f"[ISOTP] use_stack=1 txid=0x{int(self._txid):X} rxid=0x{int(self._rxid):X} "
                  f"fd={bool(self._fd)} ext={bool(self._extended)}", flush=True)

            # 진행 감시(옵션)
            first_cf_ms = _env_float("ISOTP_WD_FIRST_CF_MS", 300.0)
            cf_gap_ms   = _env_float("ISOTP_WD_CF_GAP_MS",   500.0)
            self._flow_watch = _IsoTpFlowWatch(
                src_can_id=int(self._rxid),
                first_cf_timeout_s=(first_cf_ms or 300.0) / 1000.0,
                cf_gap_timeout_s=(cf_gap_ms or 500.0) / 1000.0
            )
            try:
                self._notifier.add_listener(self._flow_watch)
            except Exception:
                pass

            # ISO-TP 스택
            try:
                self._stack = isotp.NotifierBasedCanStack(
                    self._bus, self._notifier, address=address, params=params
                )
            except Exception as e:
                raise RuntimeError(f"isotp.NotifierBasedCanStack 생성 실패: {e}")

            # 스택 처리 루프
            def _loop():
                while not self._stop_evt.is_set():
                    try:
                        self._stack.process()

                        if self._stack.available():
                            app = bytes(self._stack.recv())   # [MsgID][Payload...]
                            msg_id, body = unpack_app(app)
                            self._emit_app(msg_id, body)

                            if self._flow_watch:
                                self._flow_watch.clear()

                        if self._flow_watch and self._flow_watch.reset_needed:
                            print("[WARN] ISO-TP RX watchdog fired -> reset stack")
                            self._flow_watch.clear()
                            try:
                                reset_fn = getattr(self._stack, "reset", None)
                                if callable(reset_fn):
                                    reset_fn()
                                else:
                                    self._stack = isotp.NotifierBasedCanStack(
                                        self._bus, self._notifier, address=self._address, params=self._params
                                    )
                            except Exception:
                                time.sleep(0.05)

                        time.sleep(0.001)
                    except Exception:
                        time.sleep(0.01)

            self._thread = threading.Thread(target=_loop, daemon=False)
            self._thread.start()

        else:
            # ISO-TP 스택 미사용 시: SF 폴백 리스너 추가
            listeners = []
            self._raw_listener = _RawListener(self._q, self._emit_app, self._filter_ids)
            listeners.append(self._raw_listener)
            for L in listeners:
                try:
                    self._notifier.add_listener(L)
                except Exception:
                    pass

    def receive_packet(self, timeout_s: float = 1.0, verbose: bool = False) -> Optional[Tuple[bytes, int]]:
        try:
            return self._q.get(timeout=max(0.001, float(timeout_s)))
        except queue.Empty:
            return None

    def stop_rx(self):
        self._stop_evt.set()

        if self._thread is not None:
            try:
                self._thread.join(timeout=1.0)
            except Exception:
                pass
            self._thread = None

        self._stack = None

        if self._notifier is not None:
            for lst_attr in ("_tap", "_flow_watch", "_manual", "_raw_listener"):
                lst = getattr(self, lst_attr, None)
                if lst is not None:
                    try:
                        self._notifier.remove_listener(lst)
                    except Exception:
                        pass
                    setattr(self, lst_attr, None)

        if self._own_notifier and self._notifier is not None:
            try:
                self._notifier.stop()
            except Exception:
                pass
        self._notifier = None

        try:
            while True:
                self._q.get_nowait()
        except queue.Empty:
            pass

    def close(self):
        self.stop_rx()

# ========== 단독 테스트 ==========
if __name__ == "__main__":
    """
    단독 실행 시:
    1) 같은 폴더의 can_isotp_sender.CanIsoTpSender 를 우선 시도(버스/Notifier 공유)
    2) 실패하면 python-can으로 직접 Bus 오픈
    환경변수 예:
        set PCAN_CHANNEL=PCAN_USBBUS1
        set PCAN_FD=1
        set PCAN_F_CLOCK_MHZ=80
        set PCAN_NOM_BRP=2
        set PCAN_NOM_TSEG1=33
        set PCAN_NOM_TSEG2=6
        set PCAN_NOM_SJW=1
        set PCAN_DATA_BRP=2
        set PCAN_DATA_TSEG1=6
        set PCAN_DATA_TSEG2=1
        set PCAN_DATA_SJW=1
        set ISOTP_TXID=0xC0
        set ISOTP_RXID=0xC8
        set ISOTP_EXT=0
        set PCAN_IFG_US=3000
        set SAVE_PATH=rx_log.txt
        set APP_PREVIEW_MAX=64
        set APP_DEDUP_SEC=0.25
    """
    rx_mgr = None
    save_path = os.environ.get("SAVE_PATH", None)

    # 1) 송신기 백엔드 경로
    try:
        from can_isotp_sender import CanIsoTpSender  # same folder
        fd_flag = _env_bool("PCAN_FD", True)
        sender = CanIsoTpSender(
            channel=os.environ.get("PCAN_CHANNEL", "PCAN_USBBUS1"),
            bitrate=int(os.environ.get("PCAN_CLASSIC_BITRATE", "500000")),
            fd=bool(fd_flag),
            f_clock_mhz=_env_int("PCAN_F_CLOCK_MHZ", 80) if "PCAN_F_CLOCK" not in os.environ else None,
            f_clock=_env_int("PCAN_F_CLOCK", None),
            nom_brp=_env_int("PCAN_NOM_BRP", 2),
            nom_tseg1=_env_int("PCAN_NOM_TSEG1", 33),
            nom_tseg2=_env_int("PCAN_NOM_TSEG2", 6),
            nom_sjw=_env_int("PCAN_NOM_SJW", 1),
            data_brp=_env_int("PCAN_DATA_BRP", 2),
            data_tseg1=_env_int("PCAN_DATA_TSEG1", 6),
            data_tseg2=_env_int("PCAN_DATA_TSEG2", 1),
            data_sjw=_env_int("PCAN_DATA_SJW", 1),
            txid=_env_int("ISOTP_TXID", 0xC0),
            rxid=_env_int("ISOTP_RXID", 0xC8),
            extended=_env_bool("ISOTP_EXT", False),
            save_path=None,
        )
        sender.connect(ifg_us=int(os.environ.get("PCAN_IFG_US", "3000")))
        rx_mgr = sender._require_mgr()
        print("[INFO] Bus 확보: sender._require_mgr() 사용")
    except Exception as e:
        print(f"[WARN] CanIsoTpSender 경로 실패: {e}")

    # 2) 직접 Bus 오픈
    if rx_mgr is None:
        try:
            channel = os.environ.get("PCAN_CHANNEL", "PCAN_USBBUS1")
            fd_flag = _env_bool("PCAN_FD", True)

            if fd_flag:
                fd_kwargs = dict(
                    f_clock_mhz=_env_int("PCAN_F_CLOCK_MHZ", 80) if "PCAN_F_CLOCK" not in os.environ else None,
                    f_clock=_env_int("PCAN_F_CLOCK", None),
                    nom_brp=_env_int("PCAN_NOM_BRP", 2),
                    nom_tseg1=_env_int("PCAN_NOM_TSEG1", 33),
                    nom_tseg2=_env_int("PCAN_NOM_TSEG2", 6),
                    nom_sjw=_env_int("PCAN_NOM_SJW", 1),
                    data_brp=_env_int("PCAN_DATA_BRP", 2),
                    data_tseg1=_env_int("PCAN_DATA_TSEG1", 6),
                    data_tseg2=_env_int("PCAN_DATA_TSEG2", 1),
                    data_sjw=_env_int("PCAN_DATA_SJW", 1),
                )
                fd_kwargs = {k: v for k, v in fd_kwargs.items() if v is not None}
                bus = can.Bus(interface="pcan", channel=channel, fd=True, **fd_kwargs)
            else:
                classic_bitrate = int(os.environ.get("PCAN_CLASSIC_BITRATE", "500000"))
                bus = can.Bus(interface="pcan", channel=channel, bitrate=classic_bitrate)

            class _MiniMgr:
                def __init__(self, b, fdv: bool):
                    self._bus = b
                    self.txid = _env_int("ISOTP_TXID", 0xC0)
                    self.rxid = _env_int("ISOTP_RXID", 0xC8)
                    self.extended = _env_bool("ISOTP_EXT", False)
                    self.fd = bool(fdv)
                def get_bus(self):
                    return self._bus

            rx_mgr = _MiniMgr(bus, fd_flag)
            print("[INFO] Bus 확보: python-can 직접 오픈 (fd={}, kw={})".format(fd_flag, fd_kwargs if fd_flag else "classic"))
        except Exception as e:
            print(f"[ERR] Bus 직접 오픈 실패: {e}")
            raise SystemExit(1)

    # 수신기 시작
    rx = CanIsoTpReceiver(rx_mgr, save_path=save_path)

    def _on_pkt(payload: bytes, msg_id: int):
        try:
            txt = payload.decode("utf-8").rstrip("\r\n")
            print(f"[RX] MsgID={msg_id} {len(payload)}B ASCII: {txt}")
        except UnicodeDecodeError:
            hx = " ".join(f"{b:02X}" for b in payload)
            print(f"[RX] MsgID={msg_id} {len(payload)}B HEX  : {hx}")

    try:
        rx.start_rx(on_packet=_on_pkt)
        rx.set_raw_tap(False)   # RAW 로그 활성/비활성
        print("listening... Ctrl+C to stop")
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            rx.close()
        except Exception:
            pass
        try:
            if 'sender' in globals():
                sender.disconnect()
        except Exception:
            pass
