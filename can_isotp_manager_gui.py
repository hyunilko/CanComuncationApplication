#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
can_isotp_manager_gui.py (App-layer only logging)

- PyQt6 GUI for CAN ISO-TP
- TX: can_isotp_sender.CanIsoTpSender
- RX: can_isotp_receiver.CanIsoTpReceiver  → (완성 패킷=Application Layer: MsgID + Payload 만 전달)
- 모든 수신 패킷을 "radar packet YYYY-MM-DD HH-mm-ss.log" 로 안전 저장
  (PacketLoggerWorker: MSG ID 라인 + Payload HEX 한 줄, 패킷 사이 빈 줄)
- UI 업데이트는 반드시 메인 스레드에서만 수행 (스레드-세이프)

필요:
    pip install PyQt6 python-can can-isotp
"""

from __future__ import annotations
from typing import List, Optional, Set, Dict, Any
import sys, os, re, binascii, queue
from datetime import datetime

from PyQt6.QtCore import Qt, QThread, pyqtSignal, QObject, QEvent
from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QFileDialog, QMessageBox,
    QTextEdit, QPushButton, QVBoxLayout, QHBoxLayout, QLabel, QLineEdit,
    QCheckBox, QProgressBar, QSplitter, QListWidget, QListWidgetItem,
    QGroupBox, QSpinBox, QComboBox
)

import can

# -------- Backend (TX) --------
try:
    from can_isotp_sender import CanIsoTpSender
    _be_err = None
except Exception as e:
    CanIsoTpSender = None  # type: ignore
    _be_err = e

# -------- Receiver (RX) --------
try:
    from can_isotp_receiver import CanIsoTpReceiver
    _rx_err = None
except Exception as e:
    CanIsoTpReceiver = None  # type: ignore
    _rx_err = e


# ================= 유틸: FD 타이밍/필터 파서 =================
def _parse_filter_ids(text: str) -> Optional[Set[int]]:
    s = (text or "").strip()
    if not s:
        return None
    out: Set[int] = set()
    for tok in re.split(r"[,\s]+", s):
        if not tok:
            continue
        if "-" in tok:
            a, b = tok.split("-", 1)
            lo = int(a, 0); hi = int(b, 0)
            if lo > hi:
                lo, hi = hi, lo
            out.update(range(lo, hi + 1))
        else:
            out.add(int(tok, 0))
    return out or None


def _parse_fd_kwargs(text: str) -> Dict[str, Any]:
    if not text:
        return {}
    allowed = {
        "f_clock", "f_clock_mhz",
        "nom_brp", "nom_tseg1", "nom_tseg2", "nom_sjw",
        "data_brp", "data_tseg1", "data_tseg2", "data_sjw",
        "data_ssp_offset",
    }
    out: Dict[str, Any] = {}
    parts = [p.strip() for p in text.split(",")]
    for p in parts:
        if not p or "=" not in p:
            continue
        k, v = [x.strip() for x in p.split("=", 1)]
        if k not in allowed:
            continue
        try:
            if v.lower().startswith(("0x", "0o", "0b")):
                out[k] = int(v, 0)
            else:
                out[k] = int(v)
        except Exception:
            pass
    return out


def _sanitize_fd_kwargs(d: dict) -> dict:
    d = dict(d or {})
    d.pop("data_ssp_offset", None)
    return d


def _hexdump_bytes(b: bytes) -> str:
    return binascii.hexlify(b).decode("ascii").upper()


def _resolve_bus_like(obj):
    for name in ("bus", "_bus"):
        b = getattr(obj, name, None)
        if isinstance(b, can.BusABC):
            return b
    m = getattr(obj, "_mgr", None) or getattr(obj, "mgr", None)
    if m:
        for name in ("bus", "_bus"):
            b = getattr(m, name, None)
            if isinstance(b, can.BusABC):
                return b
        get_bus = getattr(m, "get_bus", None)
        if callable(get_bus):
            try:
                b = get_bus()
                if isinstance(b, can.BusABC):
                    return b
            except Exception:
                pass
    get_bus = getattr(obj, "get_bus", None)
    if callable(get_bus):
        try:
            b = get_bus()
            if isinstance(b, can.BusABC):
                return b
        except Exception:
            pass
    return None


# ================= 송신 작업 스레드 =================
class SendWorker(QThread):
    progress = pyqtSignal(int)
    lineSent = pyqtSignal(str)
    error = pyqtSignal(str)
    finishedOk = pyqtSignal()

    def __init__(self, sender: "CanIsoTpSender", lines: List[str]):
        super().__init__()
        self.sender = sender
        self.lines = lines
        self._stop = False

    def run(self):
        try:
            total = len(self.lines)
            if hasattr(self.sender, "send_lines"):
                i = 0
                for i, (line, _rx) in enumerate(self.sender.send_lines(self.lines), 1):
                    if self._stop:
                        break
                    self.lineSent.emit(line)
                    self.progress.emit(int(i * 100 / max(1, total)))
                self.finishedOk.emit()
                return

            for i, line in enumerate(self.lines, 1):
                if self._stop:
                    break
                self.sender.send_line(line)
                self.lineSent.emit(line)
                self.progress.emit(int(i * 100 / max(1, total)))
            self.finishedOk.emit()

        except Exception as e:
            self.error.emit(str(e))

    def stop(self):
        self._stop = True


# ================= 파일 저장 전용 워커 =================
class PacketLoggerWorker(QThread):
    error = pyqtSignal(str)
    droppedWarn = pyqtSignal(int)

    def __init__(self, path: str, flush_every: int = 20):
        super().__init__()
        self._path = path
        self._q: "queue.Queue[tuple[int, bytes]]" = queue.Queue(maxsize=8192)
        self._stop = False
        self._file = None
        self._flush_every = max(1, flush_every)
        self._write_count = 0
        self._dropped = 0

    def enqueue_msg(self, msg_id: int, payload: bytes):
        try:
            self._q.put_nowait((int(msg_id), bytes(payload)))
        except queue.Full:
            self._dropped += 1
            if self._dropped % 100 == 1:
                self.droppedWarn.emit(self._dropped)

    # RAW 입력 무시(안전장치)
    def enqueue(self, _can_id: int, _data: bytes):
        return

    def run(self):
        try:
            self._file = open(self._path, "a", encoding="utf-8", buffering=1)
        except Exception as e:
            self.error.emit(f"로그 파일 오픈 실패: {e}")
            return
        try:
            while True:
                if self._stop and self._q.empty():
                    break
                try:
                    msg_id, payload = self._q.get(timeout=0.2)
                except queue.Empty:
                    continue
                try:
                    hex_payload = _hexdump_bytes(payload)
                    total_hex_len = (1 + len(payload)) * 2
                    self._file.write(f"MSG_ID: 0x{int(msg_id):X} LEN={total_hex_len}\n")
                    self._file.write(hex_payload + "\n\n")
                    self._write_count += 1
                    if (self._write_count % self._flush_every) == 0:
                        self._file.flush()
                except Exception as e:
                    self.error.emit(f"로그 파일 쓰기 실패: {e}")
        finally:
            try:
                if self._file:
                    self._file.flush()
                    self._file.close()
            except Exception:
                pass

    def stop(self):
        self._stop = True


# ================= 폴링 수신 워커 (콜백 실패 시 폴백) =================
class PollWorker(QThread):
    rxText = pyqtSignal(str, int)      # (text, msg_id)
    rxBinary = pyqtSignal(bytes, int)  # (data, msg_id)
    error = pyqtSignal(str)

    def __init__(self, receiver: "CanIsoTpReceiver", timeout_s: float, verbose: bool):
        super().__init__()
        self.receiver = receiver
        self.timeout_s = timeout_s
        self.verbose = verbose
        self._stop = False

    def run(self):
        try:
            while not self._stop:
                pkt = self.receiver.receive_packet(timeout_s=self.timeout_s, verbose=self.verbose)
                if pkt is None:
                    self.msleep(2)
                    continue
                data, msg_id = pkt
                try:
                    text = data.decode("utf-8")
                    self.rxText.emit(text, msg_id)
                except UnicodeDecodeError:
                    self.rxBinary.emit(data, msg_id)
        except Exception as e:
            self.error.emit(str(e))

    def stop(self):
        self._stop = True


# ================= UI 브리지 =================
class UiBridge(QObject):
    rx_bin = pyqtSignal(int, object)   # (msg_id, bytes)
    rx_txt = pyqtSignal(int, str)      # (msg_id, text)
    log_msg = pyqtSignal(str)


# ================= 메인 윈도우 =================
class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("CAN ISO-TP Manager (PyQt6)")
        self.resize(1380, 900)

        if CanIsoTpSender is None:
            QMessageBox.critical(self, "오류", f"백엔드(can_isotp_sender) import 실패: {_be_err}")
        if CanIsoTpReceiver is None:
            QMessageBox.critical(self, "오류", f"수신기(can_isotp_receiver) import 실패: {_rx_err}")

        self.txWorker: Optional[SendWorker] = None
        self.pollWorker: Optional[PollWorker] = None
        self.loggerWorker: Optional[PacketLoggerWorker] = None
        self.sender: Optional["CanIsoTpSender"] = None
        self._rx_receiver: Optional[CanIsoTpReceiver] = None

        self._rx_mgr = None
        self._rx_mgr_owned = False

        # ---- UI 브리지 ----
        self.uiBus = UiBridge()
        self.uiBus.rx_bin.connect(self._on_ui_rx_bin)
        self.uiBus.rx_txt.connect(self._on_ui_rx_txt)
        self.uiBus.log_msg.connect(self._append_log)

        # ---------------- 상단: 연결/수신 설정 ----------------
        self.edChannel = QLineEdit("PCAN_USBBUS1")
        self.chkFd = QCheckBox("FD"); self.chkFd.setChecked(True)
        self.chkBRS = QCheckBox("BRS"); self.chkBRS.setChecked(False)
        self.chkExtended = QCheckBox("Extended(29-bit)"); self.chkExtended.setChecked(False)

        self.edBitrateFd = QLineEdit(
            "f_clock_mhz=80, nom_brp=2, nom_tseg1=33, nom_tseg2=6, nom_sjw=1, "
            "data_brp=2, data_tseg1=6, data_tseg2=1, data_sjw=1, data_ssp_offset=14"
        )
        self.edClassicBitrate = QLineEdit("500000")

        # ISO-TP 주소
        self.edTxId = QLineEdit("0xC0")
        self.edRxId = QLineEdit("0xC8")

        # 송신 IFG, 수신 타임아웃
        self.edIfg = QSpinBox(); self.edIfg.setRange(0, 20000); self.edIfg.setValue(3000)
        self.edTimeout = QSpinBox(); self.edTimeout.setRange(1, 120); self.edTimeout.setValue(30)
        self.chkVerbose = QCheckBox("RX verbose"); self.chkVerbose.setChecked(False)

        # EOL
        self.cbEol = QComboBox()
        self.cbEol.addItems(["none", "lf", "crlf"])
        self.cbEol.setCurrentText("none")

        # 필터
        self.edFilterIds = QLineEdit("")  # 빈칸=전체
        self.edFilterIds.setPlaceholderText("예: 0xC8, 0x200-0x20F  (빈칸=전체)")

        self.btnConnect = QPushButton("Connect")
        self.btnDisconnect = QPushButton("Disconnect"); self.btnDisconnect.setEnabled(False)

        row1 = QHBoxLayout()
        row1.addWidget(QLabel("Channel")); row1.addWidget(self.edChannel)
        row1.addSpacing(8)
        row1.addWidget(self.chkFd); row1.addWidget(self.chkBRS); row1.addWidget(self.chkExtended)
        row1.addSpacing(8)
        row1.addWidget(QLabel("FD Timings")); row1.addWidget(self.edBitrateFd)
        row1.addSpacing(8)
        row1.addWidget(QLabel("Classic(kbps)")); row1.addWidget(self.edClassicBitrate)

        row2 = QHBoxLayout()
        row2.addWidget(QLabel("TXID")); row2.addWidget(self.edTxId)
        row2.addSpacing(8)
        row2.addWidget(QLabel("RXID")); row2.addWidget(self.edRxId)
        row2.addSpacing(8)
        row2.addWidget(QLabel("IFG us")); row2.addWidget(self.edIfg)
        row2.addSpacing(8)
        row2.addWidget(QLabel("Timeout s")); row2.addWidget(self.edTimeout)
        row2.addSpacing(8)
        row2.addWidget(QLabel("EOL")); row2.addWidget(self.cbEol)
        row2.addSpacing(8)
        row2.addWidget(QLabel("Filter IDs")); row2.addWidget(self.edFilterIds)
        row2.addStretch(1)
        row2.addWidget(self.chkVerbose)
        row2.addSpacing(8)
        row2.addWidget(self.btnConnect); row2.addWidget(self.btnDisconnect)

        gbConn = QGroupBox("Connection (TX: CanIsoTpSender, RX: CanIsoTpReceiver)")
        layConn = QVBoxLayout(); layConn.addLayout(row1); layConn.addLayout(row2); gbConn.setLayout(layConn)

        # ---------------- 중앙: 좌/우 ----------------
        self.listLines = QListWidget()
        self.textEditor = QTextEdit()
        self.textEditor.setPlaceholderText("명령을 입력하거나 파일을 로드하세요. (%, 공백 스킵옵션 적용)")

        splitter = QSplitter(Qt.Orientation.Horizontal)
        splitter.addWidget(self.listLines)
        splitter.addWidget(self.textEditor)
        splitter.setStretchFactor(1, 1)

        # ---------------- 옵션/버튼 ----------------
        self.chkSkipComment = QCheckBox("주석(%) 스킵"); self.chkSkipComment.setChecked(True)
        self.chkSkipEmpty = QCheckBox("공백 스킵"); self.chkSkipEmpty.setChecked(True)

        self.btnOpen = QPushButton("파일 열기")
        self.btnSendAll = QPushButton("전체 전송")
        self.btnSendSelected = QPushButton("선택 전송")
        self.btnStop = QPushButton("중지"); self.btnStop.setEnabled(False)
        self.btnClear = QPushButton("로그 지우기")

        optRow = QHBoxLayout()
        optRow.addWidget(self.chkSkipComment)
        optRow.addWidget(self.chkSkipEmpty)
        optRow.addStretch(1)
        optRow.addWidget(self.btnOpen)
        optRow.addWidget(self.btnSendSelected)
        optRow.addWidget(self.btnSendAll)
        optRow.addWidget(self.btnStop)
        optRow.addWidget(self.btnClear)

        # ---------------- 진행/로그/콘솔 ----------------
        self.progress = QProgressBar(); self.progress.setRange(0, 100)
        self.log = QTextEdit(); self.log.setReadOnly(True)
        self.console = QTextEdit(); self.console.setReadOnly(True)
        self.console.setPlaceholderText("완성 패킷 요약(hex) 표시")
        self.consoleInput = QLineEdit()
        self.consoleInput.setPlaceholderText("단일 명령 입력. ↑/↓=히스토리, Ctrl+L=클리어")
        self.hist: List[str] = []; self.hidx: int = 0

        # 메모리/성능 보호: 최대 라인 제한
        self.console.document().setMaximumBlockCount(5000)
        self.log.document().setMaximumBlockCount(2000)

        # ---------------- 레이아웃 ----------------
        central = QWidget(); root = QVBoxLayout(central)
        root.addWidget(gbConn)
        root.addWidget(splitter, 1)
        root.addLayout(optRow)
        root.addWidget(self.progress)
        root.addWidget(self.log, 1)
        root.addWidget(self.console, 1)
        root.addWidget(self.consoleInput)
        self.setCentralWidget(central)

        # ---------------- 시그널 ----------------
        self.btnOpen.clicked.connect(self.on_open)
        self.btnSendAll.clicked.connect(self.on_send_all)
        self.btnSendSelected.clicked.connect(self.on_send_selected)
        self.btnStop.clicked.connect(self.on_stop)
        self.btnClear.clicked.connect(self.log.clear)
        self.btnConnect.clicked.connect(self.on_connect)
        self.btnDisconnect.clicked.connect(self.on_disconnect)

        self.consoleInput.returnPressed.connect(self.on_repl_enter)
        self.consoleInput.installEventFilter(self)

    # ---------- 유틸 ----------
    def _filter_lines(self, lines: List[str]) -> List[str]:
        out = []
        for s in lines:
            t = s.rstrip("\r\n")
            if self.chkSkipEmpty.isChecked() and not t.strip():
                continue
            if self.chkSkipComment.isChecked() and t.lstrip().startswith("%"):
                continue
            out.append(t)
        return out

    def _append_log(self, msg: str):
        self.log.append(msg)

    def _err(self, msg: str):
        QMessageBox.critical(self, "오류", msg)

    def _selected_eol_str(self) -> str:
        t = self.cbEol.currentText()
        if t == "lf": return "\n"
        if t == "crlf": return "\r\n"
        return ""

    # ---------- 파일 열기 ----------
    def on_open(self):
        path, _ = QFileDialog.getOpenFileName(
            self, "명령 파일 선택", "", "Text files (*.txt *.cfg *.cmd *.conf);;All files (*.*)"
        )
        if not path:
            return
        try:
            with open(path, "r", encoding="utf-8") as f:
                content = f.read()
        except UnicodeDecodeError:
            with open(path, "r", encoding="cp949", errors="ignore") as f:
                content = f.read()
        self.textEditor.setPlainText(content)
        self.listLines.clear()
        for line in self._filter_lines(content.splitlines()):
            self.listLines.addItem(QListWidgetItem(line))
        self._append_log(f"[INFO] 파일 로드: {path}")

    # ---------- Connect / Disconnect ----------
    def on_connect(self):
        if CanIsoTpSender is None or CanIsoTpReceiver is None:
            self._err("백엔드/수신기 import 실패")
            return

        if self.sender is not None or self._rx_receiver is not None or (self.pollWorker is not None):
            self._append_log("[WARN] 이미 연결되어 있습니다. 먼저 Disconnect 하세요.")
            return

        try:
            channel = self.edChannel.text().strip()
            fd = self.chkFd.isChecked()
            brs = self.chkBRS.isChecked()
            extended = self.chkExtended.isChecked()
            txid = int(self.edTxId.text().strip(), 0)
            rxid = int(self.edRxId.text().strip(), 0)
            fd_kwargs = _parse_fd_kwargs(self.edBitrateFd.text().strip())
            classic_br = int(self.edClassicBitrate.text().strip(), 0)

            self.sender = CanIsoTpSender(
                channel=channel,
                bitrate=classic_br,
                fd=fd,
                txid=txid,
                rxid=rxid,
                extended=extended,
                stmin=0,
                blocksize=0,
                rx_flowcontrol_timeout=5000,
                rx_consecutive_frame_timeout=5000,
                tx_data_length=(64 if fd else 8),
                tx_data_min_length=(64 if fd else 8),
                tx_padding=0x00,
                bitrate_switch=brs,
                blocking_send=False,
            )

            # TX connect
            try:
                self.sender.connect(ifg_us=int(self.edIfg.value()), **fd_kwargs)
                self._append_log(
                    "[INFO] TX 연결 완료"
                    + (f" (FD kwargs: {', '.join(f'{k}={v}' for k, v in fd_kwargs.items())})" if fd_kwargs else "")
                )
            except TypeError:
                self.sender.connect(ifg_us=int(self.edIfg.value()))
                if fd and fd_kwargs:
                    self._append_log("[WARN] CanIsoTpSender.connect()가 FD 타이밍 kwargs를 지원하지 않습니다.")

            # FD 타이밍 setter 시도(선택)
            for name in ("set_fd_timings", "configure_fd", "set_pcan_fd_timing"):
                if hasattr(self.sender, name) and fd and fd_kwargs:
                    try:
                        getattr(self.sender, name)(**fd_kwargs)
                        self._append_log(f"[INFO] FD 타이밍 적용: {name} 호출 성공")
                        break
                    except Exception as e:
                        self._append_log(f"[WARN] {name} 호출 실패: {e}")

            # BRS 설정(지원 시)
            for name in ("set_bitrate_switch", "set_brs", "set_can_fd_brs", "enable_brs"):
                if hasattr(self.sender, name):
                    try:
                        getattr(self.sender, name)(brs)
                    except TypeError:
                        try: getattr(self.sender, name)(bitrate_switch=brs)
                        except Exception: pass
                    break

            # ISO-TP 파라미터 강제(선택)
            if hasattr(self.sender, "set_isotp_params"):
                try:
                    self.sender.set_isotp_params(
                        tx_data_length=64 if fd else 8,
                        tx_data_min_length=64 if fd else 8,
                        tx_padding=0x00,
                        bitrate_switch=brs
                    )
                    self._append_log("[INFO] ISO-TP params applied")
                except Exception as e:
                    self._append_log(f"[WARN] set_isotp_params 실패: {e}")

            # ---- RX 매니저 확보 ----
            rx_mgr = None
            try:
                rx_mgr = self.sender._require_mgr()
            except Exception:
                rx_mgr = None

            bus_for_tap = None
            if rx_mgr is None:
                bus_from_sender = _resolve_bus_like(self.sender)
                if bus_from_sender is not None:
                    class _WrapMgr:
                        def __init__(self, b, txid, rxid, ext, fd):
                            self._bus = b
                            self.txid = txid; self.rxid = rxid
                            self.extended = ext; self.fd = fd
                        def get_bus(self): return self._bus
                    rx_mgr = _WrapMgr(bus_from_sender, txid, rxid, extended, fd)
                    bus_for_tap = bus_from_sender

            if rx_mgr is None:
                try:
                    if fd:
                        fd_kw = _sanitize_fd_kwargs(fd_kwargs)
                        try:
                            bus = can.Bus(interface="pcan", channel=channel, fd=True, **fd_kw)
                        except TypeError:
                            fd_str = ", ".join(f"{k}={v}" for k, v in fd_kw.items())
                            bus = can.Bus(interface="pcan", channel=channel, fd=True, bitrate=fd_str)
                    else:
                        bus = can.Bus(interface="pcan", channel=channel, bitrate=classic_br)
                except Exception as e:
                    self._teardown_on_error()
                    self._err(f"RX Bus 직접 오픈 실패: {e}")
                    return

                class _MiniMgr:
                    def __init__(self, b, txid, rxid, ext, fd):
                        self._bus = b
                        self.txid = txid; self.rxid = rxid
                        self.extended = ext; self.fd = fd
                    def get_bus(self): return self._bus
                rx_mgr = _MiniMgr(bus, txid, rxid, extended, fd)
                bus_for_tap = bus

            self._rx_mgr = rx_mgr
            self._rx_mgr_owned = (bus_for_tap is not None)

            # ---- 로그 파일 준비 ----
            ts = datetime.now().strftime("%Y-%m-%d %H-%M-%S")  # Windows ':' → '-'
            log_name = f"radar packet {ts}.log"
            log_path = os.path.abspath(log_name)
            self.loggerWorker = PacketLoggerWorker(log_path, flush_every=20)
            self.loggerWorker.error.connect(lambda m: self._append_log(f"[LOG-ERR] {m}"))
            self.loggerWorker.droppedWarn.connect(
                lambda n: self._append_log(f"[LOG-WARN] 저장 큐 포화로 {n}개 드롭됨 (수신은 계속 진행)")
            )
            self.loggerWorker.start()
            self._append_log(f"[INFO] 저장 파일: {log_path}")

            # ---- 수신기 ----
            filter_ids = _parse_filter_ids(self.edFilterIds.text())
            self._rx_receiver = CanIsoTpReceiver(rx_mgr, filter_can_ids=filter_ids)

            # 콜백(백그라운드 스레드) → 메인 스레드 시그널만 발생
            def _on_complete_packet(payload: bytes, msg_id: int):
                try:
                    if self.loggerWorker is not None:
                        self.loggerWorker.enqueue_msg(int(msg_id), bytes(payload))
                except Exception as e:
                    self.uiBus.log_msg.emit(f"[LOG-ERR] enqueue failed: {e}")
                self.uiBus.rx_bin.emit(int(msg_id), bytes(payload))

            started_callback = False
            try:
                self._rx_receiver.start_rx(on_packet=_on_complete_packet)
                # RAW 프레임 요약 로그 끄기 (지원 시)
                if hasattr(self._rx_receiver, "set_raw_tap"):
                    self._rx_receiver.set_raw_tap(False)
                started_callback = True
                self._append_log("[INFO] RX: 콜백 기반 연속 수신 시작(start_rx)")
            except Exception as e:
                self._append_log(f"[WARN] start_rx 실패 → 폴링으로 폴백: {e}")

            if not started_callback:
                self.pollWorker = PollWorker(
                    receiver=self._rx_receiver,
                    timeout_s=float(self.edTimeout.value()),
                    verbose=self.chkVerbose.isChecked()
                )
                self.pollWorker.rxBinary.connect(self._on_worker_rx_bin)
                self.pollWorker.rxText.connect(self._on_worker_rx_text)
                self.pollWorker.error.connect(lambda msg: self.console.append(f"[RX-ERR] {msg}"))
                self.pollWorker.start()
                self._append_log("[INFO] RX: 폴링 기반 연속 수신 시작")

            # UI 상태
            self.btnConnect.setEnabled(False)
            self.btnDisconnect.setEnabled(True)
            self.statusBar().showMessage("Connected")
            self._append_log("[INFO] 전체 연결 및 수신 시작 완료")

        except Exception as e:
            self._teardown_on_error()
            self._err(str(e))

    # PollWorker → 메인스레드 슬롯
    def _on_worker_rx_bin(self, data: bytes, _msg_id: int):
        msg_id = int(_msg_id)
        payload = bytes(data)
        try:
            if self.loggerWorker is not None:
                self.loggerWorker.enqueue_msg(msg_id, payload)
        except Exception:
            pass
        self.uiBus.rx_bin.emit(msg_id, payload)

    def _on_worker_rx_text(self, text: str, _msg_id: int):
        b = text.encode("utf-8", "replace")
        msg_id = int(_msg_id)
        try:
            if self.loggerWorker is not None:
                self.loggerWorker.enqueue_msg(msg_id, b)
        except Exception:
            pass
        self.uiBus.rx_bin.emit(msg_id, b)

    def _append_console_hex(self, msg_id: int, data: bytes):
        try:
            hexstr = _hexdump_bytes(data)
            if len(hexstr) > 4096:
                hexstr = hexstr[:4096] + "..."
            self.console.append(f"MSG ID: 0x{int(msg_id):X}\n{hexstr}\n")
        except Exception:
            pass

    # 메인스레드 UI 브리지 슬롯
    def _on_ui_rx_bin(self, msg_id: int, data_obj: object):
        try:
            data = bytes(data_obj) if not isinstance(data_obj, (bytes, bytearray)) else bytes(data_obj)
        except Exception:
            return
        self._append_console_hex(msg_id, data)
        try:
            txt = data.decode("utf-8").rstrip("\r\n")
            if txt:
                self.console.append(f"ASCII: {txt}\n")
        except UnicodeDecodeError:
            pass

    def _on_ui_rx_txt(self, msg_id: int, text: str):
        b = text.encode("utf-8", "replace")
        self._append_console_hex(msg_id, b)

    def _teardown_on_error(self):
        try:
            if self.pollWorker is not None:
                self.pollWorker.stop()
                self.pollWorker.wait(500)
                self.pollWorker = None
        except Exception:
            pass
        try:
            if self._rx_receiver is not None:
                try:
                    self._rx_receiver.stop_rx()
                except Exception:
                    pass
                try:
                    self._rx_receiver.close()
                except Exception:
                    pass
                self._rx_receiver = None
        except Exception:
            pass
        try:
            if self.loggerWorker is not None:
                self.loggerWorker.stop()
                self.loggerWorker.wait(1000)
                self.loggerWorker = None
        except Exception:
            pass

        self._rx_mgr = None
        self._rx_mgr_owned = False
        try:
            if self.sender is not None:
                if hasattr(self.sender, "stop_repl"):
                    try: self.sender.stop_repl()  # type: ignore[attr-defined]
                    except Exception: pass
                self.sender.disconnect()
        except Exception:
            pass
        self.sender = None

    def on_disconnect(self):
        try:
            if self.pollWorker is not None:
                self.pollWorker.stop()
                self.pollWorker.wait(1000)
                self.pollWorker = None
            if self._rx_receiver is not None:
                try:
                    self._rx_receiver.close()
                except Exception:
                    pass
                self._rx_receiver = None
        except Exception:
            pass

        try:
            if self.loggerWorker is not None:
                self.loggerWorker.stop()
                self.loggerWorker.wait(1500)
                self.loggerWorker = None
        except Exception:
            pass

        self._rx_mgr = None
        self._rx_mgr_owned = False

        if self.sender is not None:
            try:
                if hasattr(self.sender, "stop_repl"):
                    try: self.sender.stop_repl()  # type: ignore[attr-defined]
                    except Exception: pass
                self.sender.disconnect()
            except Exception as e:
                self._err(str(e))
            self.sender = None

        self.btnConnect.setEnabled(True)
        self.btnDisconnect.setEnabled(False)
        self.statusBar().showMessage("Disconnected")
        self._append_log("[INFO] 연결 해제")

    # ---------- 전송 ----------
    def _start_send(self, lines: List[str]):
        if self.sender is None:
            self._err("연결되어 있지 않습니다. 먼저 Connect 하세요.")
            return
        if not lines:
            self._append_log("[WARN] 전송할 라인이 없습니다.")
            return
        eol = self._selected_eol_str()
        lines2 = [s + eol for s in lines]

        self.progress.setValue(0)
        self.btnSendAll.setEnabled(False)
        self.btnSendSelected.setEnabled(False)
        self.btnStop.setEnabled(True)

        self.txWorker = SendWorker(self.sender, lines2)
        self.txWorker.progress.connect(self.progress.setValue)
        self.txWorker.lineSent.connect(self._on_tx_line_sent)
        self.txWorker.error.connect(self._on_worker_error)
        self.txWorker.finishedOk.connect(self._on_worker_done)
        self.txWorker.start()

    def _on_worker_error(self, msg: str):
        self._append_log(f"[ERR] {msg}")
        self._reset_buttons()

    def _on_worker_done(self):
        self._append_log("[INFO] 전송 완료")
        self._reset_buttons()

    def _reset_buttons(self):
        self.btnSendAll.setEnabled(True)
        self.btnSendSelected.setEnabled(True)
        self.btnStop.setEnabled(False)
        self.progress.setValue(100)
        self.txWorker = None

    def on_send_all(self):
        lines = self._filter_lines(self.textEditor.toPlainText().splitlines())
        self._start_send(lines)

    def on_send_selected(self):
        sel = self.listLines.selectedItems()
        if sel:
            lines = [it.text() for it in sel]
        else:
            cursor = self.textEditor.textCursor()
            lines = [cursor.block().text()]
        self._start_send(self._filter_lines(lines))

    def on_stop(self):
        if self.txWorker is not None:
            self.txWorker.stop()
            self._append_log("[INFO] 중지 요청됨")

    # ---------- REPL ----------
    def on_repl_enter(self):
        if self.sender is None:
            self.console.append("[ERR] 연결되어 있지 않습니다.")
            return
        cmd = self.consoleInput.text().strip()
        if not cmd:
            return
        self.hist.append(cmd); self.hidx = len(self.hist)
        self.console.append(f">>> {cmd}")
        try:
            eol = self._selected_eol_str()
            payload = (cmd + eol).encode("utf-8", "strict")
            self.sender.send_line(cmd + eol)
            self._append_log(f"[TX] {cmd}")
            self._append_log(f"[TX-HEX {len(payload)}B] {_hexdump_bytes(payload)}")
        except Exception as e:
            self.console.append(f"[ERR] {e}")
        finally:
            self.consoleInput.clear()

    # ↑/↓ 히스토리, Ctrl+L 클리어
    def eventFilter(self, obj, ev):
        if obj is self.consoleInput and ev.type() == QEvent.Type.KeyPress:
            key = ev.key(); mod = ev.modifiers()
            if key == Qt.Key.Key_L and (mod & Qt.KeyboardModifier.ControlModifier):
                self.console.clear(); self.consoleInput.clear(); return True
            if key == Qt.Key.Key_Up:
                if self.hist:
                    self.hidx = max(0, self.hidx - 1)
                    self.consoleInput.setText(self.hist[self.hidx]); return True
            if key == Qt.Key.Key_Down:
                if self.hist:
                    self.hidx = min(len(self.hist), self.hidx + 1)
                    if self.hidx == len(self.hist): self.consoleInput.clear()
                    else: self.consoleInput.setText(self.hist[self.hidx])
                    return True
        return super().eventFilter(obj, ev)

    # 창 닫을 때 안전 정리
    def closeEvent(self, ev):
        try:
            self.on_disconnect()
        except Exception:
            pass
        super().closeEvent(ev)

    def _on_tx_line_sent(self, s: str):
        try:
            b = s.encode("utf-8", "strict")
        except Exception:
            b = s.encode("utf-8", "replace")
        text_preview = s.rstrip("\r\n")
        hex_preview  = _hexdump_bytes(b)
        self._append_log(f"[TX] {text_preview}")
        self._append_log(f"[TX-HEX {len(b)}B] {hex_preview}")


def main():
    app = QApplication(sys.argv)
    w = MainWindow()
    w.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
