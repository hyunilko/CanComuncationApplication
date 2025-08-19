#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
can_sender-receiver_gui.py (stable)

- PyQt6 GUI for CAN CLI
- TX: can_cli_command_sender.CanCliCommandSender
- RX: can_data_receiver.CanDataReceiver (조립 완료 '완성 패킷' 수신)
- 모든 수신 패킷을 "radar packet YYYY-MM-DD HH-mm-ss.log" 로 안전 저장
- UI 업데이트는 반드시 메인 스레드에서만 수행 (스레드-세이프)

pip install PyQt6
"""

from __future__ import annotations
from typing import List, Optional, Set
import sys, os, re, binascii, queue
from datetime import datetime

from PyQt6.QtCore import Qt, QThread, pyqtSignal, QObject, QEvent
from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QFileDialog, QMessageBox,
    QTextEdit, QPushButton, QVBoxLayout, QHBoxLayout, QLabel, QLineEdit,
    QCheckBox, QProgressBar, QSplitter, QListWidget, QListWidgetItem, QGroupBox, QSpinBox
)

# -------- Backend (TX) --------
try:
    from can_cli_command_sender import CanCliCommandSender
    _be_err = None
except Exception as e:
    CanCliCommandSender = None  # type: ignore
    _be_err = e

# -------- Receiver (RX) --------
try:
    from can_data_receiver import CanDataReceiver
    _rx_err = None
except Exception as e:
    CanDataReceiver = None  # type: ignore
    _rx_err = e

# -------- PCANManager (fallback용) --------
try:
    from pcan_manager import PCANManager
except Exception:
    PCANManager = None  # type: ignore


# ================= 유틸: 필터 파서 =================
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


# ================= 송신 작업 스레드 =================
class SendWorker(QThread):
    progress = pyqtSignal(int)
    lineSent = pyqtSignal(str)
    error = pyqtSignal(str)
    finishedOk = pyqtSignal()

    def __init__(self, sender: "CanCliCommandSender", lines: List[str]):
        super().__init__()
        self.sender = sender
        self.lines = lines
        self._stop = False

    def run(self):
        try:
            total = len(self.lines)
            try:
                self.sender.send_lines(self.lines)
                for i, line in enumerate(self.lines, 1):
                    if self._stop:
                        break
                    self.lineSent.emit(line)
                    self.progress.emit(int(i * 100 / max(1, total)))
                self.finishedOk.emit()
                return
            except Exception:
                pass
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
    droppedWarn = pyqtSignal(int)  # 누적 드롭 수 알림

    def __init__(self, path: str, flush_every: int = 20):
        super().__init__()
        self._path = path
        self._q: "queue.Queue[tuple[int, bytes]]" = queue.Queue(maxsize=4096)
        self._stop = False
        self._file = None
        self._flush_every = max(1, flush_every)
        self._write_count = 0
        self._dropped = 0

    def enqueue(self, can_id: int, data: bytes):
        try:
            self._q.put_nowait((can_id, data))
        except queue.Full:
            self._dropped += 1
            if self._dropped % 100 == 1:
                self.droppedWarn.emit(self._dropped)

    def run(self):
        try:
            self._file = open(self._path, "a", encoding="ascii", buffering=1)
        except Exception as e:
            self.error.emit(f"로그 파일 오픈 실패: {e}")
            return
        try:
            while not self._stop:
                try:
                    can_id, data = self._q.get(timeout=0.2)
                except queue.Empty:
                    continue
                try:
                    hexstr = binascii.hexlify(data).decode("ascii").upper()
                    self._file.write(f"CAN ID: 0x{can_id:X}\n")
                    self._file.write(hexstr + "\n\n")  # 패킷 구분용 빈 줄
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
    rxText = pyqtSignal(str, int)
    rxBinary = pyqtSignal(bytes, int)
    error = pyqtSignal(str)

    def __init__(self, receiver: "CanDataReceiver", timeout_s: float, verbose: bool):
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
                data, can_id = pkt
                try:
                    text = data.decode("utf-8")
                    self.rxText.emit(text, can_id)
                except UnicodeDecodeError:
                    self.rxBinary.emit(data, can_id)
        except Exception as e:
            self.error.emit(str(e))

    def stop(self):
        self._stop = True


# ================= UI 브리지(메인 스레드에서만 UI 갱신) =================
class UiBridge(QObject):
    rx_bin = pyqtSignal(int, object)   # (can_id, bytes)
    rx_txt = pyqtSignal(int, str)      # (can_id, text)
    log_msg = pyqtSignal(str)


# ================= 메인 윈도우 =================
class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("CAN Sender/Receiver (Framed) - PyQt6")
        self.resize(1280, 860)

        if CanCliCommandSender is None:
            QMessageBox.critical(self, "오류", f"백엔드(can_cli_command_sender) import 실패: {_be_err}")
        if CanDataReceiver is None:
            QMessageBox.critical(self, "오류", f"수신기(can_data_receiver) import 실패: {_rx_err}")

        self.txWorker: Optional[SendWorker] = None
        self.pollWorker: Optional[PollWorker] = None
        self.loggerWorker: Optional[PacketLoggerWorker] = None
        self.sender: Optional["CanCliCommandSender"] = None
        self._rx_receiver: Optional[CanDataReceiver] = None

        self._rx_mgr_owned = False
        self._rx_mgr = None

        # ---- UI 브리지(메인스레드 시그널로만 UI수정) ----
        self.uiBus = UiBridge()
        self.uiBus.rx_bin.connect(self._on_ui_rx_bin)
        self.uiBus.rx_txt.connect(self._on_ui_rx_txt)
        self.uiBus.log_msg.connect(self._append_log)

        # ---------------- 상단: 연결/수신 설정 ----------------
        self.edChannel = QLineEdit("PCAN_USBBUS1")
        self.edBitrate = QLineEdit(
            "f_clock=80000000,nom_brp=2,nom_tseg1=33,nom_tseg2=6,nom_sjw=1,"
            "data_brp=2,data_tseg1=6,data_tseg2=1,data_sjw=1,data_ssp_offset=14"
        )
        self.edIfg = QSpinBox(); self.edIfg.setRange(0, 20000); self.edIfg.setValue(3000)
        self.edFilterIds = QLineEdit("")  # 빈칸=전체
        self.edFilterIds.setPlaceholderText("예: 0xD1, 0x200-0x20F  (빈칸=전체)")
        self.edTimeout = QSpinBox(); self.edTimeout.setRange(1, 120); self.edTimeout.setValue(30)
        self.chkVerbose = QCheckBox("RX verbose"); self.chkVerbose.setChecked(False)

        self.btnConnect = QPushButton("Connect")
        self.btnDisconnect = QPushButton("Disconnect"); self.btnDisconnect.setEnabled(False)

        row = QHBoxLayout()
        row.addWidget(QLabel("Channel")); row.addWidget(self.edChannel)
        row.addSpacing(10)
        row.addWidget(QLabel("BitrateFD")); row.addWidget(self.edBitrate)
        row.addSpacing(10)
        row.addWidget(QLabel("IFG us")); row.addWidget(self.edIfg)
        row.addSpacing(10)
        row.addWidget(QLabel("Filter IDs")); row.addWidget(self.edFilterIds)
        row.addSpacing(10)
        row.addWidget(QLabel("Timeout s")); row.addWidget(self.edTimeout)
        row.addWidget(self.chkVerbose)
        row.addStretch(1)
        row.addWidget(self.btnConnect); row.addWidget(self.btnDisconnect)

        gbConn = QGroupBox("Connection (TX: CanCliCommandSender, RX: CanDataReceiver)")
        layConn = QVBoxLayout(); layConn.addLayout(row); gbConn.setLayout(layConn)

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
        if CanCliCommandSender is None or CanDataReceiver is None:
            self._err("백엔드/수신기 import 실패")
            return

        if self.sender is not None or self._rx_receiver is not None or (self.pollWorker is not None):
            self._append_log("[WARN] 이미 연결되어 있습니다. 먼저 Disconnect 하세요.")
            return

        try:
            # TX
            self.sender = CanCliCommandSender(
                channel=self.edChannel.text().strip(),
                bitrate_fd=self.edBitrate.text().strip(),
            )
            self.sender.connect(ifg_us=int(self.edIfg.value()))
            self._append_log("[INFO] TX 연결 완료")

            # RX 매니저
            rx_mgr = None
            try:
                rx_mgr = self.sender._require_mgr()  # type: ignore[attr-defined]
            except Exception:
                rx_mgr = None
            if rx_mgr is None:
                if PCANManager is None:
                    self._err("PCANManager 로드 실패: RX 매니저 생성 불가")
                    try:
                        self.sender.disconnect()
                    except Exception:
                        pass
                    self.sender = None
                    return
                rx_mgr = PCANManager()
                rx_mgr.open(self.edChannel.text().strip(),
                            self.edBitrate.text().strip(),
                            ifg_us=int(self.edIfg.value()))
                self._rx_mgr_owned = True
                self._append_log("[INFO] 별도 RX 매니저 오픈")
            else:
                self._rx_mgr_owned = False
            self._rx_mgr = rx_mgr

            # 로그 파일 준비
            ts = datetime.now().strftime("%Y-%m-%d %H-%M-%S")  # Windows ':' 금지 → '-' 사용
            log_name = f"radar packet {ts}.log"
            log_path = os.path.abspath(log_name)
            self.loggerWorker = PacketLoggerWorker(log_path, flush_every=20)
            self.loggerWorker.error.connect(lambda m: self._append_log(f"[LOG-ERR] {m}"))
            self.loggerWorker.droppedWarn.connect(
                lambda n: self._append_log(f"[LOG-WARN] 저장 큐 포화로 {n}개 드롭됨 (수신은 계속 진행)")
            )
            self.loggerWorker.start()
            self._append_log(f"[INFO] 저장 파일: {log_path}")

            # 수신기
            filter_ids = _parse_filter_ids(self.edFilterIds.text())
            self._rx_receiver = CanDataReceiver(rx_mgr, filter_can_ids=filter_ids)

            # 콜백(백그라운드 스레드) → 메인 스레드 신호만 발생
            def _on_complete_packet(data: bytes, can_id: int):
                try:
                    if self.loggerWorker is not None:
                        self.loggerWorker.enqueue(int(can_id), bytes(data))
                except Exception:
                    pass
                # UI는 직접 만지지 않고, 신호로 전달
                try:
                    self.uiBus.rx_bin.emit(int(can_id), bytes(data))
                except Exception:
                    pass

            # 콜백 기반 연속 수신 시도
            started_callback = False
            try:
                self._rx_receiver.start_rx(on_packet=_on_complete_packet)
                started_callback = True
                self._append_log("[INFO] RX: 콜백 기반 연속 수신 시작(start_rx)")
            except Exception as e:
                self._append_log(f"[WARN] start_rx 실패 → 폴링으로 폴백: {e}")

            # 폴링 폴백 경로: PollWorker 시그널 → 메인스레드 슬롯
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
    def _on_worker_rx_bin(self, data: bytes, can_id: int):
        try:
            if self.loggerWorker is not None:
                self.loggerWorker.enqueue(int(can_id), bytes(data))
        except Exception:
            pass
        self.uiBus.rx_bin.emit(int(can_id), bytes(data))

    def _on_worker_rx_text(self, text: str, can_id: int):
        b = text.encode("utf-8", "replace")
        try:
            if self.loggerWorker is not None:
                self.loggerWorker.enqueue(int(can_id), b)
        except Exception:
            pass
        self.uiBus.rx_txt.emit(int(can_id), text)

    def _append_console_hex(self, can_id: int, data: bytes):
        try:
            hexstr = binascii.hexlify(data).decode("ascii").upper()
            if len(hexstr) > 256:
                hexstr = hexstr[:256] + "..."
            self.console.append(f"CAN ID: 0x{can_id:X}\n{hexstr}\n")
        except Exception:
            pass

    # 메인스레드 UI 브리지 슬롯
    def _on_ui_rx_bin(self, can_id: int, data_obj: object):
        try:
            data = bytes(data_obj) if not isinstance(data_obj, (bytes, bytearray)) else bytes(data_obj)
        except Exception:
            return
        self._append_console_hex(can_id, data)

    def _on_ui_rx_txt(self, can_id: int, text: str):
        self.console.append(f"CAN ID: 0x{can_id:X}\n{text}\n")

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
        if self._rx_mgr_owned and self._rx_mgr is not None:
            try:
                if hasattr(self._rx_mgr, "close"):
                    self._rx_mgr.close()  # type: ignore[attr-defined]
            except Exception:
                pass
            self._rx_mgr_owned = False
            self._rx_mgr = None
        try:
            if self.sender is not None:
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

        if self._rx_mgr_owned and self._rx_mgr is not None:
            try:
                if hasattr(self._rx_mgr, "close"):
                    self._rx_mgr.close()  # type: ignore[attr-defined]
            except Exception:
                pass
            self._rx_mgr_owned = False
            self._rx_mgr = None

        if self.sender is not None:
            try:
                try:
                    self.sender.stop_repl()
                except Exception:
                    pass
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
        self.progress.setValue(0)
        self.btnSendAll.setEnabled(False)
        self.btnSendSelected.setEnabled(False)
        self.btnStop.setEnabled(True)

        self.txWorker = SendWorker(self.sender, lines)
        self.txWorker.progress.connect(self.progress.setValue)
        self.txWorker.lineSent.connect(lambda s: self._append_log(f"[TX] {s}"))
        self.txWorker.error.connect(self._on_worker_error)
        self.txWorker.finishedOk.connect(self._on_worker_done)
        self.txWorker.start()

    def _on_worker_error(self, msg: str):
        self._append_log(f"[ERR] {msg}")
        self._reset_buttons()

    def _on_worker_done(self):
        self._append_log("[INFO] 전송 완료]")
        self._reset_buttons()

    def _reset_buttons(self):
        self.btnSendAll.setEnabled(True)
        the_same = True
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
            self.sender.send_line(cmd)
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


def main():
    app = QApplication(sys.argv)
    w = MainWindow()
    w.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
