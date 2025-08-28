#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
can_isotp_sender.py

PCAN + ISO-TP 기반 CLI 커맨드 송수신기 (Class 형태)
 - python-can 의 PCAN 인터페이스로 버스 오픈
 - can-isotp(pylessard)의 NotifierBasedCanStack 사용(세그멘테이션/재조립)
 - Application Layer: [Msg ID | Payload...] 프레이밍 적용
   * CLI Command용 Msg ID = 250(0xFA)
 - 파일 저장 포맷(옵션 --save):
   <Msg ID(10진수)>\n<payload(UTF-8 또는 HEX)>\n

필요 패키지:
    pip install python-can can-isotp
"""

from __future__ import annotations
from typing import Optional, Iterable, Tuple
import argparse
import logging
import time
import sys
import os

import can
import isotp

log = logging.getLogger("can_cli_isotp")

# ==== Application Layer 정의 ====
APP_MSG_ID_CLI = 0xFA  # 250

def pack_app(msg_id: int, payload: bytes) -> bytes:
    if payload is None:
        payload = b""
    return bytes([msg_id & 0xFF]) + payload

def try_unpack_app(pdu: bytes) -> Tuple[int, bytes]:
    """pdu가 [MsgID][Payload...] 형식이면 (msg_id, payload) 반환, 아니면 ( -1, 원문 )."""
    if not pdu:
        return (-1, b"")
    return (pdu[0], pdu[1:])

def payload_to_text_or_hex(payload: bytes) -> str:
    try:
        return payload.decode("utf-8")
    except UnicodeDecodeError:
        return "HEX:" + " ".join(f"{b:02X}" for b in payload)


# ---------------- 유틸 ----------------
def _hexdump(b: bytes) -> str:
    return " ".join(f"{x:02X}" for x in b)


def _build_pcan_fd_kwargs(
    f_clock: Optional[int],
    f_clock_mhz: Optional[int],
    nom_brp: Optional[int], nom_tseg1: Optional[int], nom_tseg2: Optional[int], nom_sjw: Optional[int],
    data_brp: Optional[int], data_tseg1: Optional[int], data_tseg2: Optional[int], data_sjw: Optional[int],
    data_ssp_offset: Optional[int]
) -> dict:
    defaults = dict(
        f_clock_mhz=80,
        nom_brp=2,  nom_tseg1=33, nom_tseg2=6, nom_sjw=1,
        data_brp=2, data_tseg1=6,  data_tseg2=1, data_sjw=1,
    )
    kw = defaults.copy()
    if f_clock is not None:
        kw.pop("f_clock_mhz", None)
        kw["f_clock"] = int(f_clock)
    if f_clock_mhz is not None:
        kw.pop("f_clock", None)
        kw["f_clock_mhz"] = int(f_clock_mhz)

    def _maybe_set(name: str, v: Optional[int]):
        if v is not None:
            kw[name] = int(v)

    for k, v in (
        ("nom_brp", nom_brp),
        ("nom_tseg1", nom_tseg1),
        ("nom_tseg2", nom_tseg2),
        ("nom_sjw", nom_sjw),
        ("data_brp", data_brp),
        ("data_tseg1", data_tseg1),
        ("data_tseg2", data_tseg2),
        ("data_sjw", data_sjw),
    ):
        _maybe_set(k, v)

    if data_ssp_offset is not None:
        kw["data_ssp_offset"] = int(data_ssp_offset)

    return kw


def _to_payload_bytes(s: str, eol: str) -> bytes:
    if eol == "none":
        return s.encode("utf-8", errors="strict")
    elif eol == "lf":
        return (s + "\n").encode("utf-8", errors="strict")
    elif eol == "crlf":
        return (s + "\r\n").encode("utf-8", errors="strict")
    else:
        raise ValueError("invalid eol")


# ---------------- 클래스 ----------------
class CanIsoTpSender:
    """
    PCAN 버스 + ISO-TP 스택을 관리하고, 문자열/바이트 전송 및 응답 수신을 제공.
    Application Layer: [Msg ID | Payload...] 적용 (CLI Msg ID=0xFA)
    """

    txid: int
    rxid: int
    extended: bool
    fd: bool

    def __init__(
        self,
        *,
        channel: str = "PCAN_USBBUS1",
        bitrate: int = 500000,
        fd: bool = True,
        # FD timings
        f_clock: Optional[int] = None,
        f_clock_mhz: Optional[int] = None,
        nom_brp: Optional[int] = None, nom_tseg1: Optional[int] = None, nom_tseg2: Optional[int] = None, nom_sjw: Optional[int] = None,
        data_brp: Optional[int] = None, data_tseg1: Optional[int] = None, data_tseg2: Optional[int] = None, data_sjw: Optional[int] = None,
        data_ssp_offset: Optional[int] = None,
        # ISO-TP addressing
        txid: int = 0xC0,
        rxid: int = 0xC8,
        extended: bool = False,
        # ISO-TP params
        stmin: int = 0,
        blocksize: int = 8,
        tx_data_length: int = 8,
        tx_data_min_length: Optional[int] = None,
        override_receiver_stmin: Optional[float] = None,
        rx_flowcontrol_timeout: int = 1000,
        rx_consecutive_frame_timeout: int = 1000,
        tx_padding: Optional[int] = 0x00,
        bitrate_switch: bool = False,
        default_target_address_type: isotp.TargetAddressType = isotp.TargetAddressType.Physical,
        blocking_send: bool = False,
        # App behavior
        eol: str = "none",
        timeout_s: float = 2.0,
        ifg_us: int = 3000,
        # Save
        save_path: Optional[str] = None,
    ):
        self.channel = channel
        self.bitrate = int(bitrate)
        self.fd = bool(fd)

        self._fd_kwargs = _build_pcan_fd_kwargs(
            f_clock, f_clock_mhz,
            nom_brp, nom_tseg1, nom_tseg2, nom_sjw,
            data_brp, data_tseg1, data_tseg2, data_sjw,
            data_ssp_offset
        )

        self.txid = int(txid)
        self.rxid = int(rxid)
        self.extended = bool(extended)

        self.params = dict(
            stmin=int(stmin),
            blocksize=int(blocksize),
            tx_data_length=int(tx_data_length if (fd and tx_data_length == 8) is False else (64 if fd else 8)),
            tx_data_min_length=(None if tx_data_min_length is None else int(tx_data_min_length)),
            override_receiver_stmin=override_receiver_stmin,
            rx_flowcontrol_timeout=int(rx_flowcontrol_timeout),
            rx_consecutive_frame_timeout=int(rx_consecutive_frame_timeout),
            tx_padding=(None if tx_padding is None else int(tx_padding)),
            can_fd=self.fd,
            bitrate_switch=bool(bitrate_switch),
            default_target_address_type=default_target_address_type,
            blocking_send=bool(blocking_send),
        )

        self.eol = eol
        self.timeout_s = float(timeout_s)
        self.ifg_us = int(ifg_us)
        self.save_path = save_path

        self.bus: Optional[can.BusABC] = None
        self.notifier: Optional[can.Notifier] = None
        self.stack: Optional[isotp.NotifierBasedCanStack] = None

    # ---- 매니저 호환 메서드 ----
    def get_bus(self) -> can.BusABC:
        if self.bus is None:
            raise RuntimeError("Bus not opened")
        return self.bus

    def _require_mgr(self):
        return self

    # ---- 저장 ----
    def _save_record(self, msg_id: int, payload: bytes):
        if not self.save_path:
            return
        try:
            with open(self.save_path, "a", encoding="utf-8", newline="") as f:
                f.write(f"{int(msg_id)}\n")
                f.write(payload_to_text_or_hex(payload))
                f.write("\n")
        except Exception as e:
            log.warning("save failed: %s", e)

    # ---- 연결/해제 ----
    def connect(self, ifg_us: Optional[int] = None, **ignored_kwargs):
        if ifg_us is not None:
            self.ifg_us = int(ifg_us)

        if not self.fd:
            self.bus = can.Bus(
                interface="pcan",
                channel=self.channel,
                bitrate=self.bitrate,
                receive_own_messages=False,
            )
        else:
            self.bus = can.Bus(
                interface="pcan",
                channel=self.channel,
                fd=True,
                receive_own_messages=False,
                **self._fd_kwargs
            )

        addr_mode = isotp.AddressingMode.Normal_29bits if self.extended else isotp.AddressingMode.Normal_11bits
        address = isotp.Address(addr_mode, txid=self.txid, rxid=self.rxid)

        def _on_isotp_error(err: isotp.IsoTpError):
            log.warning("IsoTP error: %s: %s", err.__class__.__name__, str(err))

        self.notifier = can.Notifier(self.bus, listeners=[], timeout=0.05)
        self.stack = isotp.NotifierBasedCanStack(self.bus, self.notifier, address=address,
                                                 error_handler=_on_isotp_error, params=self.params)
        self.stack.start()
        log.info("Opened PCAN bus: channel=%s, fd=%s", self.channel, self.fd)
        log.info("ISO-TP started. txid=0x%X rxid=0x%X (%s-bit)",
                 self.txid, self.rxid, "29" if self.extended else "11")

    def disconnect(self):
        if self.stack is not None:
            try: self.stack.stop()
            except Exception: pass
            self.stack = None
        if self.notifier is not None:
            try: self.notifier.stop()
            except Exception: pass
            self.notifier = None
        if self.bus is not None:
            try: self.bus.shutdown()
            except Exception: pass
            self.bus = None
        log.info("Closed.")

    # ---- 송신/수신 ----
    def send_bytes(self, payload: bytes, *, timeout_s: Optional[float] = None, blocking: Optional[bool] = None) -> None:
        if self.stack is None:
            raise RuntimeError("Stack not started")
        if payload is None:
            payload = b""

        # Application Layer 프레이밍
        app = pack_app(APP_MSG_ID_CLI, payload)

        timeout = self.timeout_s if timeout_s is None else float(timeout_s)
        blocking_send = self.params.get("blocking_send", False) if blocking is None else bool(blocking)

        if blocking_send:
            self.stack.send(app, timeout=timeout)
            log.debug("TX(APP %dB): %s", len(app), _hexdump(app))
        else:
            self.stack.send(app)
            log.debug("TX(APP %dB): %s", len(app), _hexdump(app))
            t0 = time.monotonic()
            while self.stack.transmitting() and (time.monotonic() - t0 < timeout):
                time.sleep(0.002)

        # TX 기록 (요청 포맷)
        self._save_record(APP_MSG_ID_CLI, payload)

    def read_response(self, *, timeout_s: Optional[float] = None) -> Optional[Tuple[int, bytes]]:
        """응답을 Application Layer로 언패킹하여 (msg_id, payload) 반환."""
        if self.stack is None:
            raise RuntimeError("Stack not started")
        timeout = self.timeout_s if timeout_s is None else float(timeout_s)
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            if self.stack.available():
                pdu = self.stack.recv()
                msg_id, body = try_unpack_app(bytes(pdu))
                # RX 기록
                self._save_record(msg_id if msg_id >= 0 else 0, body)
                return (msg_id, body)
            time.sleep(0.002)
        return None

    # ---- 편의 함수 ----
    def send_line(self, line: str, *, eol: Optional[str] = None, timeout_s: Optional[float] = None) -> Optional[Tuple[int, bytes]]:
        payload = _to_payload_bytes(line, eol or self.eol)
        self.send_bytes(payload, timeout_s=timeout_s)
        return self.read_response(timeout_s=timeout_s)

    def send_lines(self, lines: Iterable[str], *, eol: Optional[str] = None, timeout_s: Optional[float] = None) -> Iterable[Tuple[str, Optional[Tuple[int,bytes]]]]:
        gap = max(0.0, self.ifg_us / 1_000_000.0)
        for line in lines:
            rx = self.send_line(line, eol=eol, timeout_s=timeout_s)
            if gap:
                time.sleep(gap)
            yield (line, rx)

    def stop_repl(self):
        pass


# ---------------- 스크립트 진입점 ----------------
def _build_argparser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="CAN CLI command sender over ISO-TP (PCAN) - class based (AppLayer framed)")

    # PCAN
    p.add_argument("--channel", default="PCAN_USBBUS1")
    p.add_argument("--bitrate", type=int, default=500000)

    # FD on/off
    p.add_argument("--fd", dest="fd", action="store_true", default=True)
    p.add_argument("--no-fd", dest="fd", action="store_false")

    # FD timings
    p.add_argument("--f_clock", type=int)
    p.add_argument("--f_clock_mhz", type=int)
    p.add_argument("--nom_brp", type=int)
    p.add_argument("--nom_tseg1", type=int)
    p.add_argument("--nom_tseg2", type=int)
    p.add_argument("--nom_sjw", type=int)
    p.add_argument("--data_brp", type=int)
    p.add_argument("--data_tseg1", type=int)
    p.add_argument("--data_tseg2", type=int)
    p.add_argument("--data_sjw", type=int)
    p.add_argument("--data_ssp_offset", type=int)

    # ISO-TP addressing
    p.add_argument("--txid", type=lambda x: int(x, 0), default=0xC0)
    p.add_argument("--rxid", type=lambda x: int(x, 0), default=0xC8)
    p.add_argument("--extended", action="store_true")

    # ISO-TP params
    p.add_argument("--stmin", type=lambda x: int(x, 0), default=0)
    p.add_argument("--bs", type=int, default=8)
    p.add_argument("--tx_data_length", type=int, default=8)
    p.add_argument("--tx_data_min_length", type=int, default=None)
    p.add_argument("--override_receiver_stmin", type=float, default=None)
    p.add_argument("--rx_fc_timeout", type=int, default=1000)
    p.add_argument("--rx_cf_timeout", type=int, default=5000)
    p.add_argument("--tx_padding", type=lambda x: int(x, 0), default=0x00)
    p.add_argument("--can_fd_brs", action="store_true")

    # App behavior
    p.add_argument("--eol", choices=["none", "lf", "crlf"], default="crlf")
    p.add_argument("--timeout", type=float, default=2.0)
    p.add_argument("--blocking_send", action="store_true")
    p.add_argument("--verbose", action="store_true")
    p.add_argument("--ifg-us", dest="ifg_us", type=int, default=3000)

    # Save file
    p.add_argument("--save", dest="save_path", default=None, help="메시지 기록 파일 경로 (2줄/메시지: MsgID, Payload)")

    return p


def main() -> int:
    parser = _build_argparser()
    args = parser.parse_args()

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )
    timeout_s = float(args.timeout)
    log.info("Args: channel=%s fd=%s bitrate=%s txid=0x%X rxid=0x%X save=%s",
             args.channel, args.fd, args.bitrate, args.txid, args.rxid, args.save_path)

    sender = CanIsoTpSender(
        channel=args.channel,
        bitrate=args.bitrate,
        fd=args.fd,
        f_clock=args.f_clock,
        f_clock_mhz=args.f_clock_mhz,
        nom_brp=args.nom_brp, nom_tseg1=args.nom_tseg1, nom_tseg2=args.nom_tseg2, nom_sjw=args.nom_sjw,
        data_brp=args.data_brp, data_tseg1=args.data_tseg1, data_tseg2=args.data_tseg2, data_sjw=args.data_sjw,
        data_ssp_offset=args.data_ssp_offset,
        txid=args.txid, rxid=args.rxid, extended=args.extended,
        stmin=args.stmin, blocksize=args.bs,
        tx_data_length=args.tx_data_length,
        tx_data_min_length=args.tx_data_min_length,
        override_receiver_stmin=args.override_receiver_stmin,
        rx_flowcontrol_timeout=args.rx_fc_timeout,
        rx_consecutive_frame_timeout=args.rx_cf_timeout,
        tx_padding=args.tx_padding,
        bitrate_switch=args.can_fd_brs,
        blocking_send=args.blocking_send,
        eol=args.eol,
        timeout_s=timeout_s,
        ifg_us=args.ifg_us,
        save_path=args.save_path,
    )

    try:
        sender.connect(ifg_us=args.ifg_us)
        log.info("ISO-TP ready. (%s-bit, %s)", "29" if args.extended else "11", "FD" if args.fd else "Classic")

        print("\n[ENTER] 명령을 입력하세요.  (종료: Ctrl+C)")
        while True:
            try:
                line = input("> ")
            except EOFError:
                break

            rx = sender.send_line(line, timeout_s=timeout_s)
            if rx is None:
                print("(no response)")
            else:
                msg_id, body = rx
                if msg_id < 0:
                    print(f"RX[PDU {len(body)}B] (no App header). HEX: {_hexdump(body)}")
                else:
                    try:
                        ascii_text = body.decode("utf-8").rstrip("\r\n")
                        print(f"RX[MsgID={msg_id} {len(body)}B] ASCII: {ascii_text}")
                    except UnicodeDecodeError:
                        print(f"RX[MsgID={msg_id} {len(body)}B] HEX  : {_hexdump(body)}")

    except KeyboardInterrupt:
        print("\nKeyboardInterrupt - shutting down")
    except Exception as e:
        log.error("Error: %s", e)
        return 2
    finally:
        sender.disconnect()

    return 0


if __name__ == "__main__":
    sys.exit(main())
