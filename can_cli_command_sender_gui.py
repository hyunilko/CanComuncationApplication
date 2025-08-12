#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
can_cli_command_sender_gui.py
- PyQt6 GUI for CAN CLI
- Backend: can_cli_command_sender.py (uses pcan_manager.PCANManager under the hood)
- 프레이밍/조립은 전부 백엔드에서 처리. GUI는 표시/전송/파일/REPL만 담당.

필요 패키지:
    pip install PyQt6
"""

from __future__ import annotations
from typing import List, Optional

import sys
from PyQt6.QtCore import Qt, QThread, pyqtSignal, QObject, QEvent
from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QFileDialog, QMessageBox,
    QTextEdit, QPushButton, QVBoxLayout, QHBoxLayout, QLabel, QLineEdit,
    QCheckBox, QProgressBar, QSplitter, QListWidget, QListWidgetItem, QGroupBox
)

# -------- Backend 고정 import --------
try:
    import can_cli_command_sender as backend  # must provide: connect/disconnect/send_line/send_lines/start_repl/stop_repl
    _be_err = None
except Exception as e:
    backend = None
    _be_err = e


# -------- 송신 작업 스레드 --------
class SendWorker(QThread):
    progress = pyqtSignal(int)
    lineSent = pyqtSignal(str)
    error = pyqtSignal(str)
    finishedOk = pyqtSignal()

    def __init__(self, lines: List[str]):
        super().__init__()
        self.lines = lines
        self._stop = False

    def run(self):
        try:
            total = len(self.lines)
            # 배치 먼저 시도
            try:
                backend.send_lines(self.lines)  # type: ignore[attr-defined]
                for i, line in enumerate(self.lines, 1):
                    if self._stop:
                        break
                    self.lineSent.emit(line)
                    self.progress.emit(int(i * 100 / max(1, total)))
                self.finishedOk.emit()
                return
            except Exception:
                pass

            # 단건 루프
            for i, line in enumerate(self.lines, 1):
                if self._stop:
                    break
                backend.send_line(line)  # type: ignore[attr-defined]
                self.lineSent.emit(line)
                self.progress.emit(int(i * 100 / max(1, total)))
            self.finishedOk.emit()
        except Exception as e:
            self.error.emit(str(e))

    def stop(self):
        self._stop = True


# -------- REPL 수신 신호 브리지 (스레드→UI) --------
class ReplBridge(QObject):
    rxText = pyqtSignal(str, int)  # text, can_id


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("CAN CLI Command Sender (PyQt6)")
        self.resize(1180, 820)

        if backend is None:
            QMessageBox.critical(self, "오류", f"백엔드(can_cli_command_sender) import 실패: {_be_err}")

        self.worker: Optional[SendWorker] = None

        # ---------------- 상단: 연결 설정 ----------------
        self.edChannel = QLineEdit("PCAN_USBBUS1")
        self.edBitrate = QLineEdit(
            "f_clock=80000000,nom_brp=2,nom_tseg1=33,nom_tseg2=6,nom_sjw=1,"
            "data_brp=2,data_tseg1=6,data_tseg2=1,data_sjw=1,data_ssp_offset=14"
        )
        self.btnConnect = QPushButton("Connect")
        self.btnDisconnect = QPushButton("Disconnect")
        self.btnDisconnect.setEnabled(False)

        row = QHBoxLayout()
        row.addWidget(QLabel("Channel"))
        row.addWidget(self.edChannel)
        row.addSpacing(10)
        row.addWidget(QLabel("BitrateFD"))
        row.addWidget(self.edBitrate)
        row.addStretch(1)
        row.addWidget(self.btnConnect)
        row.addWidget(self.btnDisconnect)

        gbConn = QGroupBox("Connection (pcan_manager → can_cli_command_sender)")
        layConn = QVBoxLayout()
        layConn.addLayout(row)
        gbConn.setLayout(layConn)

        # ---------------- 중앙: 좌(리스트) / 우(에디터) ----------------
        self.listLines = QListWidget()
        self.textEditor = QTextEdit()
        self.textEditor.setPlaceholderText("여기에 명령을 입력하거나, 파일을 열어 미리보세요. (%, 공백은 스킵 옵션 적용)")

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

        # ---------------- 진행/로그 ----------------
        self.progress = QProgressBar(); self.progress.setRange(0, 100)
        self.log = QTextEdit(); self.log.setReadOnly(True)

        # ---------------- REPL 콘솔 ----------------
        self.console = QTextEdit(); self.console.setReadOnly(True)
        self.console.setPlaceholderText("REPL 콘솔 출력 (장치 응답 표시: 프레이밍 조립 완료 문자열)")
        self.consoleInput = QLineEdit()
        self.consoleInput.setPlaceholderText("REPL 명령 입력 후 Enter.  ↑/↓: 히스토리,  Ctrl+L: 콘솔 클리어")
        self.hist: List[str] = []
        self.hidx: int = 0

        # 브리지(스레드→UI)
        self.replBridge = ReplBridge()
        self.replBridge.rxText.connect(self._on_repl_rx_text)

        # ---------------- 레이아웃 구성 ----------------
        central = QWidget()
        root = QVBoxLayout(central)
        root.addWidget(gbConn)
        root.addWidget(splitter, 1)
        root.addLayout(optRow)
        root.addWidget(self.progress)
        root.addWidget(self.log, 1)
        root.addWidget(self.console, 1)
        root.addWidget(self.consoleInput)
        self.setCentralWidget(central)

        # ---------------- 시그널 연결 ----------------
        self.btnOpen.clicked.connect(self.on_open)
        self.btnSendAll.clicked.connect(self.on_send_all)
        self.btnSendSelected.clicked.connect(self.on_send_selected)
        self.btnStop.clicked.connect(self.on_stop)
        self.btnClear.clicked.connect(self.log.clear)
        self.btnConnect.clicked.connect(self.on_connect)
        self.btnDisconnect.clicked.connect(self.on_disconnect)

        self.consoleInput.returnPressed.connect(self.on_repl_enter)
        self.consoleInput.installEventFilter(self)

    # ================= 유틸 =================
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

    # ================= 파일 열기 =================
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

    # ================= Connect/Disconnect =================
    def on_connect(self):
        if backend is None:
            self._err(f"백엔드 import 실패: {_be_err}")
            return
        try:
            backend.connect(self.edChannel.text().strip(), self.edBitrate.text().strip())
            # REPL 수신 시작: 백엔드에서 조립된 텍스트 단위로 콜백됨
            def _on_rx(text: str, can_id: int):
                self.replBridge.rxText.emit(text, can_id)

            if hasattr(backend, "start_repl"):
                backend.start_repl(_on_rx)  # type: ignore[attr-defined]
            self.btnConnect.setEnabled(False)
            self.btnDisconnect.setEnabled(True)
            self.statusBar().showMessage("Connected")
            self._append_log("[INFO] 연결 완료")
        except Exception as e:
            self._err(str(e))

    def on_disconnect(self):
        if backend is None:
            return
        try:
            if hasattr(backend, "stop_repl"):
                try:
                    backend.stop_repl()  # type: ignore[attr-defined]
                except Exception:
                    pass
            backend.disconnect()  # type: ignore[attr-defined]
            self.btnConnect.setEnabled(True)
            self.btnDisconnect.setEnabled(False)
            self.statusBar().showMessage("Disconnected")
            self._append_log("[INFO] 연결 해제")
        except Exception as e:
            self._err(str(e))

    # ================= 전송 =================
    def _start_send(self, lines: List[str]):
        if backend is None:
            self._err(f"백엔드 import 실패: {_be_err}")
            return
        if not lines:
            self._append_log("[WARN] 전송할 라인이 없습니다.")
            return
        self.progress.setValue(0)
        self.btnSendAll.setEnabled(False)
        self.btnSendSelected.setEnabled(False)
        self.btnStop.setEnabled(True)

        self.worker = SendWorker(lines)
        self.worker.progress.connect(self.progress.setValue)
        self.worker.lineSent.connect(lambda s: self._append_log(f"[TX] {s}"))
        self.worker.error.connect(self._on_worker_error)
        self.worker.finishedOk.connect(self._on_worker_done)
        self.worker.start()

    def _on_worker_error(self, msg: str):
        self._append_log(f"[ERR] {msg}")
        self._reset_buttons()

    def _on_worker_done(self):
        self._append_log("[INFO] 전송 완료]")
        self._reset_buttons()

    def _reset_buttons(self):
        self.btnSendAll.setEnabled(True)
        self.btnSendSelected.setEnabled(True)
        self.btnStop.setEnabled(False)
        self.progress.setValue(100)
        self.worker = None

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
        if self.worker is not None:
            self.worker.stop()
            self._append_log("[INFO] 중지 요청됨")

    # ================= REPL 콘솔 =================
    def _on_repl_rx_text(self, text: str, can_id: int):
        # 백엔드에서 프레이밍 조립 완료된 한 덩어리 문자열이 들어옵니다.
        self.console.append(f"[{hex(can_id)}] {text}")

    def on_repl_enter(self):
        if backend is None:
            return
        cmd = self.consoleInput.text().strip()
        if not cmd:
            return
        # 히스토리
        self.hist.append(cmd)
        self.hidx = len(self.hist)
        # 에코
        self.console.append(f">>> {cmd}")
        # 전송 (빠른 동기 호출; 무거우면 QThread 재사용 가능)
        try:
            backend.send_line(cmd)  # type: ignore[attr-defined]
        except Exception as e:
            self.console.append(f"[ERR] {e}")
        finally:
            self.consoleInput.clear()

    # ↑/↓ 히스토리, Ctrl+L 클리어
    def eventFilter(self, obj, ev):
        if obj is self.consoleInput and ev.type() == QEvent.Type.KeyPress:
            key = ev.key()
            mod = ev.modifiers()
            if key == Qt.Key.Key_L and (mod & Qt.KeyboardModifier.ControlModifier):
                self.console.clear()
                self.consoleInput.clear()
                return True
            if key == Qt.Key.Key_Up:
                if self.hist:
                    self.hidx = max(0, self.hidx - 1)
                    self.consoleInput.setText(self.hist[self.hidx])
                return True
            if key == Qt.Key.Key_Down:
                if self.hist:
                    self.hidx = min(len(self.hist), self.hidx + 1)
                    if self.hidx == len(self.hist):
                        self.consoleInput.clear()
                    else:
                        self.consoleInput.setText(self.hist[self.hidx])
                return True
        return super().eventFilter(obj, ev)


def main():
    app = QApplication(sys.argv)
    w = MainWindow()
    w.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
