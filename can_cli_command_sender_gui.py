# can_cli_command_sender_gui.py
"""
GUI sender for CLI-over-CAN FD using PCANBasic.
- Frame format: [HDR(1)] + [PAYLOAD(63)] = 64B (FD DLC=15)
  * HDR bit7: 1=LAST, 0=MIDDLE
  * HDR bit6..0: MIDDLE→SEQ(0..127), LAST→last_len(1..63)
"""

from __future__ import annotations

import threading
import tkinter as tk
from tkinter import ttk, messagebox, filedialog
from PCANBasic import *  # TPCANMsgFD, PCAN_USBBUS*, PCAN_ERROR_*, etc.
from pcan_manager import make_fd_msg, write_fd_blocking

CHUNK = 63

DEFAULT_BITRATE_FD = (
    "f_clock=80000000,"
    "nom_brp=2,nom_tseg1=33,nom_tseg2=6,nom_sjw=1,"
    "data_brp=2,data_tseg1=6,data_tseg2=1,data_sjw=1,data_ssp_offset=14"
)


def iter_canfd_frames(cmd: str, chunk: int = CHUNK):
    """63B씩 분할. 마지막 프레임: last_len(1..63)+0패딩."""
    # 장치에서 ASCII 기대라면 ascii 고정. (필요시 utf-8로 바꿔도 됨)
    payload = cmd.encode("ascii", errors="ignore")
    n = len(payload)
    if n == 0:
        return
    pos = 0
    seq = 0
    while (n - pos) > chunk:
        hdr = seq & 0x7F  # MIDDLE
        frame = bytes([hdr]) + payload[pos:pos + chunk]
        assert len(frame) == 1 + chunk == 64
        yield frame
        pos += chunk
        seq = (seq + 1) & 0x7F

    last_len = n - pos
    if last_len <= 0 or last_len > chunk:
        last_len = chunk
    hdr = 0x80 | (last_len & 0x7F)  # LAST
    pad = bytes(chunk - last_len)
    frame = bytes([hdr]) + payload[pos:pos + last_len] + pad
    assert len(frame) == 1 + chunk == 64
    yield frame


def resolve_channel(name: str):
    """문자열 채널명을 PCAN 상수로 변환."""
    try:
        return globals()[name]
    except KeyError:
        return PCAN_USBBUS1


class CanCliSenderGUI(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("CAN-FD CLI Sender (PCAN)")
        self.geometry("900x600")

        self._pcan = None
        self._channel = PCAN_USBBUS1
        self._is_std = True
        self._use_brs = True
        self._ifg_us = 1500
        self._max_retry = 100
        self._bitrate_fd = DEFAULT_BITRATE_FD
        self._canid = 0xC0

        self._build_widgets()

    # ------------- UI -------------
    def _build_widgets(self):
        frm_top = ttk.LabelFrame(self, text="Connection / Options")
        frm_top.pack(fill="x", padx=8, pady=6)

        # Channel
        ttk.Label(frm_top, text="Channel:").grid(row=0, column=0, padx=6, pady=4, sticky="e")
        self.cbo_channel = ttk.Combobox(frm_top, width=16, state="readonly",
                                        values=[f"PCAN_USBBUS{i}" for i in range(1, 9)])
        self.cbo_channel.current(0)
        self.cbo_channel.grid(row=0, column=1, padx=6, pady=4, sticky="w")

        # CAN ID
        ttk.Label(frm_top, text="CAN ID:").grid(row=0, column=2, padx=6, pady=4, sticky="e")
        self.ent_canid = ttk.Entry(frm_top, width=10)
        self.ent_canid.insert(0, "0xC0")
        self.ent_canid.grid(row=0, column=3, padx=6, pady=4, sticky="w")

        # Extended / BRS
        self.var_ext = tk.BooleanVar(value=False)
        self.var_brs = tk.BooleanVar(value=True)
        ttk.Checkbutton(frm_top, text="Extended (29-bit)", variable=self.var_ext).grid(row=0, column=4, padx=6, pady=4)
        ttk.Checkbutton(frm_top, text="Use BRS", variable=self.var_brs).grid(row=0, column=5, padx=6, pady=4)

        # IFG / Retry
        ttk.Label(frm_top, text="IFG (us):").grid(row=1, column=0, padx=6, pady=4, sticky="e")
        self.ent_ifg = ttk.Entry(frm_top, width=10)
        self.ent_ifg.insert(0, "1500")
        self.ent_ifg.grid(row=1, column=1, padx=6, pady=4, sticky="w")

        ttk.Label(frm_top, text="Max Retry:").grid(row=1, column=2, padx=6, pady=4, sticky="e")
        self.ent_retry = ttk.Entry(frm_top, width=10)
        self.ent_retry.insert(0, "100")
        self.ent_retry.grid(row=1, column=3, padx=6, pady=4, sticky="w")

        # Bitrate FD
        ttk.Label(frm_top, text="Bitrate FD:").grid(row=2, column=0, padx=6, pady=4, sticky="e")
        self.ent_bitrate = ttk.Entry(frm_top, width=80)
        self.ent_bitrate.insert(0, DEFAULT_BITRATE_FD)
        self.ent_bitrate.grid(row=2, column=1, columnspan=5, padx=6, pady=4, sticky="w")

        # Buttons
        self.btn_connect = ttk.Button(frm_top, text="Connect", command=self.on_connect)
        self.btn_disconnect = ttk.Button(frm_top, text="Disconnect", command=self.on_disconnect, state="disabled")
        self.btn_connect.grid(row=0, column=6, padx=6, pady=4, sticky="w")
        self.btn_disconnect.grid(row=1, column=6, padx=6, pady=4, sticky="w")

        # Command panel
        frm_cmd = ttk.LabelFrame(self, text="Command")
        frm_cmd.pack(fill="both", expand=True, padx=8, pady=6)

        self.txt_cmd = tk.Text(frm_cmd, height=8)
        self.txt_cmd.pack(fill="both", expand=True, padx=6, pady=6)

        frm_cmd_opts = ttk.Frame(frm_cmd)
        frm_cmd_opts.pack(fill="x", padx=6, pady=2)

        self.var_split = tk.BooleanVar(value=True)
        ttk.Checkbutton(frm_cmd_opts, text="Send each line separately", variable=self.var_split).pack(side="left", padx=4)

        ttk.Button(frm_cmd_opts, text="Load From File...", command=self.on_load_file).pack(side="left", padx=4)
        ttk.Button(frm_cmd_opts, text="Clear", command=lambda: self.txt_cmd.delete("1.0", "end")).pack(side="left", padx=4)
        self.btn_send = ttk.Button(frm_cmd_opts, text="Send", command=self.on_send, state="disabled")
        self.btn_send.pack(side="right", padx=4)

        # Log
        frm_log = ttk.LabelFrame(self, text="Log")
        frm_log.pack(fill="both", expand=True, padx=8, pady=6)
        self.txt_log = tk.Text(frm_log, height=12, state="disabled")
        self.txt_log.pack(fill="both", expand=True, padx=6, pady=6)

        self.protocol("WM_DELETE_WINDOW", self.on_close)

    # ------------- Connection -------------
    def on_connect(self):
        if self._pcan is not None:
            return
        try:
            self._channel = resolve_channel(self.cbo_channel.get())
            self._canid = int(self.ent_canid.get(), 0)
            self._is_std = not self.var_ext.get()
            self._use_brs = self.var_brs.get()
            self._ifg_us = int(self.ent_ifg.get())
            self._max_retry = int(self.ent_retry.get())
            self._bitrate_fd = self.ent_bitrate.get().strip()

            pcan = PCANBasic()
            bitrate_fd = TPCANBitrateFD(self._bitrate_fd.encode("ascii"))
            st = pcan.InitializeFD(self._channel, bitrate_fd)
            if st != PCAN_ERROR_OK:
                raise RuntimeError(f"InitializeFD failed: 0x{int(st):X}")

            self._pcan = pcan
            self._set_connected(True)
            self._log("[OK] Connected")
        except Exception as e:
            self._log(f"[ERR] {e}")
            messagebox.showerror("Connect Error", str(e))

    def on_disconnect(self):
        if self._pcan is None:
            return
        try:
            self._pcan.Uninitialize(PCAN_NONEBUS)
        except Exception:
            pass
        self._pcan = None
        self._set_connected(False)
        self._log("[OK] Disconnected")

    def _set_connected(self, connected: bool):
        self.btn_connect.config(state="disabled" if connected else "normal")
        self.btn_disconnect.config(state="normal" if connected else "disabled")
        self.btn_send.config(state="normal" if connected else "disabled")
        self.cbo_channel.config(state="disabled" if connected else "readonly")
        self.ent_canid.config(state="disabled" if connected else "normal")
        self.ent_bitrate.config(state="disabled" if connected else "normal")
        self.ent_ifg.config(state="disabled" if connected else "normal")
        self.ent_retry.config(state="disabled" if connected else "normal")
        # Extended/BRS는 연결 중에도 토글할 수 있게 둠
        # self.var_ext / self.var_brs are always enabled

    # ------------- Sending -------------
    def on_send(self):
        if self._pcan is None:
            messagebox.showwarning("Not Connected", "Please connect first.")
            return

        text = self.txt_cmd.get("1.0", "end").strip("\n")
        if not text.strip():
            return

        if self.var_split.get():
            cmds = [ln.strip() for ln in text.splitlines() if ln.strip()]
        else:
            cmds = [text.strip()]

        # 전송은 백그라운드 스레드에서
        threading.Thread(target=self._send_worker, args=(cmds,), daemon=True).start()

    def _send_worker(self, cmds):
        try:
            for cmd in cmds:
                for frame in iter_canfd_frames(cmd, CHUNK):
                    msg = make_fd_msg(self._canid, frame, is_std=self._is_std, use_brs=self._use_brs)
                    write_fd_blocking(self._pcan, self._channel, msg,
                                      ifg_us=self._ifg_us, max_retry=self._max_retry)
                self._log(f"[TX] {cmd}")
        except Exception as e:
            self._log(f"[ERR] {e}")

    # ------------- Helpers -------------
    def on_load_file(self):
        path = filedialog.askopenfilename(
            title="Open text file",
            filetypes=[("Text files", "*.txt"), ("All files", "*.*")]
        )
        if not path:
            return
        try:
            with open(path, "r", encoding="utf-8") as f:
                content = f.read()
            self.txt_cmd.delete("1.0", "end")
            self.txt_cmd.insert("1.0", content)
        except Exception as e:
            messagebox.showerror("File Error", str(e))

    def _log(self, msg: str):
        self.txt_log.config(state="normal")
        self.txt_log.insert("end", msg + "\n")
        self.txt_log.see("end")
        self.txt_log.config(state="disabled")

    def on_close(self):
        try:
            if self._pcan is not None:
                self._pcan.Uninitialize(PCAN_NONEBUS)
        finally:
            self.destroy()


def main():
    app = CanCliSenderGUI()
    app.mainloop()


if __name__ == "__main__":
    main()
