#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
can_custom_tp_send_profilecfg.py
- Sends an Application PDU for Custom-TP framing using existing can_custom_tp_sender.py
- APP_PDU = [ MsgID(1B) | Payload ]
- Payload = MMWave_ProfileComCfg (packed LE, 11 bytes)

Usage (defaults are sensible):
  python can_custom_tp_send_profilecfg.py --msg-id 0x24

Transport:
  - Uses CanCustomTpSender (PCAN + 64B frames, HDR+DATA[63] framing)
  - Reuses the existing frame builder & PCAN sending path for compatibility

Requirements:
  - can_custom_tp_sender.py (from your project)
  - mmwave_profile_cfg.py   (from your project)
  - PCAN runtime (as used by your existing tools)
"""

from __future__ import annotations
import argparse
import logging
import time
import sys

# Import project modules
try:
    from can_custom_tp_sender import CanCustomTpSender, DEFAULT_CHANNEL, DEFAULT_BITRATE_FD, DEFAULT_FRAME_GAP_US, DEFAULT_CMD_GAP_MS
except Exception as e:
    print(f"[ERR] can_custom_tp_sender import failed: {e}", file=sys.stderr)
    raise

try:
    from mmwave_profile_cfg import MMWave_ProfileComCfg, hexdump, WIRE_SIZE
except Exception as e:
    print(f"[ERR] mmwave_profile_cfg import failed: {e}", file=sys.stderr)
    raise


log = logging.getLogger("can_custom_tp_send_profilecfg")


def build_args():
    p = argparse.ArgumentParser(description="Send MMWave_ProfileComCfg as Application PDU over Custom-TP (PCAN)")
    # -------- Application level --------
    p.add_argument("--msg-id", type=lambda x: int(x, 0), default=0x24, help="Application Msg ID (1B, default 0x24)")

    # -------- Profile fields (default values can be changed) --------
    p.add_argument("--samp",  type=int,   default=1,    help="digOutputSampRate (uint8, default 1)")
    p.add_argument("--bits",  type=int,   default=2,    help="digOutputBitsSel  (uint8, default 2)")
    p.add_argument("--fir",   type=int,   default=3,    help="dfeFirSel         (uint8, default 3)")
    p.add_argument("--adc",   type=int,   default=4,    help="numOfAdcSamples   (uint16, default 4)")
    p.add_argument("--ramp",  type=float, default=5.0,  help="chirpRampEndTimeus (float, default 5.0)")
    p.add_argument("--rxhpf", type=int,   default=6,    help="chirpRxHpfSel     (uint8, default 6)")
    p.add_argument("--mimo",  type=int,   default=7,    help="chirpTxMimoPatSel (uint8, default 7)")

    # -------- Transport/Link params (reuse your existing defaults) --------
    p.add_argument("--channel", default=DEFAULT_CHANNEL, help="PCAN channel (e.g., PCAN_USBBUS1)")
    p.add_argument("--bitrate-fd", default=DEFAULT_BITRATE_FD, help="PCAN FD bitrate string")
    p.add_argument("--id", dest="can_id", type=lambda x: int(x, 0), default=0xC0, help="11-bit CAN ID (link-layer)")
    p.add_argument("--ifg-us", type=int, default=DEFAULT_FRAME_GAP_US, help="inter-frame gap (us)")
    p.add_argument("--cmd-gap-ms", type=int, default=DEFAULT_CMD_GAP_MS, help="gap after one logical command (ms)")

    # -------- Logging/behavior --------
    p.add_argument("--verbose", action="store_true", help="verbose logging")
    p.add_argument("--sleep-after", type=float, default=0.05, help="sleep after TX to let PCAN drain (seconds)")
    return p


def main() -> int:
    args = build_args().parse_args()

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )

    # Build payload
    cfg = MMWave_ProfileComCfg(
        digOutputSampRate=args.samp,
        digOutputBitsSel=args.bits,
        dfeFirSel=args.fir,
        numOfAdcSamples=args.adc,
        chirpRampEndTimeus=args.ramp,
        chirpRxHpfSel=args.rxhpf,
        chirpTxMimoPatSel=args.mimo,
    )
    payload = cfg.to_bytes()
    if len(payload) != WIRE_SIZE:
        log.error("Unexpected payload size: %d (expected %d)", len(payload), WIRE_SIZE)
        return 2

    # Compose APP_PDU = [MsgID(1B)] + [Payload]
    app_msg_id = args.msg_id & 0xFF
    app_pdu = bytes([app_msg_id]) + payload

    log.info("APP_PDU MsgID=0x%02X Payload(%dB)=%s", app_msg_id, len(payload), hexdump(payload))

    # Prepare sender (reuse the existing Custom-TP PCAN sender path)
    sender = CanCustomTpSender(
        channel=args.channel,
        bitrate_fd=args.bitrate_fd,
        can_id_11bit=args.can_id,
        app_msg_id=app_msg_id,   # not strictly needed since we pass full PDU below
        append_lf=False,         # binary payload, do NOT append LF
        frame_gap_us=args.ifg_us,
        cmd_gap_ms=args.cmd_gap_ms,
    )

    try:
        sender.connect(ifg_us=args.ifg_us)
        log.info("Connected: channel=%s id=0x%X", sender.channel, sender.can_id_11bit)

        # Use existing internal helper to split into 64B frames
        try:
            frames = sender._build_frames_from_app_pdu(app_pdu)  # type: ignore[attr-defined]
        except AttributeError:
            log.error("Your can_custom_tp_sender.py does not provide _build_frames_from_app_pdu")
            return 3

        # Send frames via the same PCAN manager path used in your working sender
        mgr = sender._require_mgr()  # type: ignore[attr-defined]
        for frame in frames:
            if hasattr(mgr, "send_bytes"):
                mgr.send_bytes(sender.can_id_11bit, frame)     # type: ignore[attr-defined]
            elif hasattr(mgr, "send_frame"):
                mgr.send_frame(sender.can_id_11bit, frame)     # type: ignore[attr-defined]
            elif hasattr(mgr, "write"):
                mgr.write(sender.can_id_11bit, frame)          # type: ignore[attr-defined]
            else:
                raise RuntimeError("PCANManager has no send method (send_bytes/send_frame/write)")

        log.info("TX done: %d frames, APP_PDU %d bytes", len(frames), len(app_pdu))
        time.sleep(max(0.0, float(args.sleep_after)))

    finally:
        try:
            sender.disconnect()
        except Exception:
            pass

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
