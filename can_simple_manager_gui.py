#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
can_simple_manager_gui.py  (optimized for CanSimpleSender/Receiver + PCANManager)

- PyQt6 GUI
- TX: can_simple_sender.CanSimpleSender (내부적으로 PCANManager 사용)
- RX: can_simple_receiver.CanSimpleReceiver (TX의 같은 PCANManager 재사용)
- 수신기는 hexdump 프린트 + "radar packet YYYY-MM-DD HH-mm-ss.log" 저장 수행
- GUI는 수신 프레임을 스레드 세이프하게 콘솔에 표시(파일 저장은 중복 방지 위해 GUI에서 하지 않음)

필요:
    pip install PyQt6
"""

from __future__ import annotations
from typing import List, Optional, Set
import re

import sys, os, binascii
from PyQt6.QtCore import Qt, QThread, pyqtSignal, QObject, QEvent
from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QFileDialog, QMessageBox,
    QTextEdit, QPushButton, QVBoxLayout, QHBoxLayout, QLabel, QLineEdit,
    QCheckBox, QProgressBar, QSplitter, QListWidget, QListWidgetItem, QGroupBox, QSpinBox
)

# -------- Backend (TX) --------
try:
    from can_simple_sender import CanSimpleSender
    _be_err = None
except Exception as e:
    CanSimpleSender = None  # type: ignore
    _be_err = e

# -------- Receiver (RX: simple raw) --------
try:
    from can_simple_receiver import CanSimpleReceiver
    _rx_err = None
except Exception as e:
    CanSimpleReceiver = None  # type: ignore
    _rx_err = e


# ================= 송신 작업 스레드 =================
class SendWorker(QThread):
    progress = pyqtSignal(int)
    lineSent = pyqtSignal(str)
    error = pyqtSignal(str)
    finishedOk = pyqtSignal()

    def __init__(self, sender: "CanSimpleSender", lines: List[str]):
        super().__init__()
        self.sender = sender
        self.lines = lines
        self._stop = False

    def run(self):
        try:
            total = len(self.lines)
            # 일괄 전송 시도
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
            # 라인별 전송 폴백
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


# ================= 수신 폴링 워커 (RX 콜백 실패 시만 사용) =================
class ReceiveWorker(QThread):
    rxText = pyqtSignal(str, int)      # 텍스트, CAN ID
    rxBinary = pyqtSignal(bytes, int)  # 바이너리, CAN ID
    error = pyqtSignal(str)

    def __init__(self, receiver: "CanSimpleReceiver", timeout_s: float, verbose: bool):
        super().__init__()
        self.receiver = receiver
        self.timeout_s = timeout_s
        self.verbose = verbose
        self._stop = False

    def run(self):
        try:
            while not self._stop:
                try:
                    pkt = self.receiver.receive_packet(timeout_s=self.timeout_s, verbose=self.verbose)
                except TypeError:
                    pkt = self.receiver.receive_packet(self.timeout_s)
                if pkt is None:
                    self.msleep(2)
                    continue
                if isinstance(pkt, tuple):
                    data, can_id = pkt[0], pkt[1] if len(pkt) > 1 else -1
                else:
                    data, can_id = pkt, -1
                # 텍스트/바이너리 분기
                try:
                    text = data.decode("utf-8")
                    self.rxText.emit(text, can_id if isinstance(can_id, int) else -1)
                except UnicodeDecodeError:
                    self.rxBinary.emit(data, can_id if isinstance(can_id, int) else -1)
        except Exception as e:
            self.error.emit(str(e))

    def stop(self):
        self._stop = True


# ================= UI 브릿지 (스레드 세이프 UI 업데이트) =================
class UiBridge(QObject):
    rx_bin = pyqtSignal(int, object)   # (can_id, bytes)
    rx_txt = pyqtSignal(int, str)      # (can_id, text)
    log_msg = pyqtSignal(str)          # 로그창 메시지


# ================= 메인 윈도우 =================
class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("CAN Sender/Receiver (Simple) - PyQt6")
        self.resize(1280, 860)

        if CanSimpleSender is None:
            QMessageBox.critical(self, "오류", f"송신기(can_simple_sender) import 실패: {_be_err}")
        if CanSimpleReceiver is None:
            QMessageBox.critical(self, "오류", f"수신기(can_simple_receiver) import 실패: {_rx_err}")

        self.txWorker: Optional[SendWorker] = None
        self.rxWorker: Optional[ReceiveWorker] = None
        self.sender: Optional["CanSimpleSender"] = None
        self._rx_receiver: Optional[CanSimpleReceiver] = None

        # ---- UI 브릿지 ----
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
        self.edFilterId = QLineEdit("")      # 공란=전체
        self.edFilterId.setPlaceholderText("예) 0xD1, 0xD2  또는  0x100-0x10F  (공란=전체)")
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
        row.addWidget(QLabel("Filter IDs")); row.addWidget(self.edFilterId)
        row.addSpacing(10)
        row.addWidget(QLabel("Timeout s")); row.addWidget(self.edTimeout)
        row.addWidget(self.chkVerbose)
        row.addStretch(1)
        row.addWidget(self.btnConnect); row.addWidget(self.btnDisconnect)

        gbConn = QGroupBox("Connection (TX: CanSimpleSender, RX: CanSimpleReceiver)")
        layConn = QVBoxLayout(); layConn.addLayout(row); gbConn.setLayout(layConn)

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

        # ---------------- 콘솔(수신/REPL) ----------------
        self.console = QTextEdit(); self.console.setReadOnly(True)
        self.console.setPlaceholderText("수신 데이터 hexdump 출력 영역")
        self.consoleInput = QLineEdit()
        self.consoleInput.setPlaceholderText("단일 명령 입력 후 Enter.  ↑/↓: 히스토리,  Ctrl+L: 콘솔 클리어")
        self.hist: List[str] = []; self.hidx: int = 0

        # ---------------- 레이아웃 구성 ----------------
        central = QWidget(); root = QVBoxLayout(central)
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

    # 다중/범위 CAN ID 파서
    def _parse_filter_ids(self) -> Optional[Set[int]]:
        """
        입력 예:
          ""                   -> None (전체 수신)
          "0xD1"               -> {0xD1}
          "0xD1,0xD2 0xD3"     -> {0xD1, 0xD2, 0xD3}
          "0x100-0x10F"        -> {0x100..0x10F}
          "0xD1, 0x200-0x205"  -> 혼합 가능
        """
        s = self.edFilterId.text().strip()
        if not s:
            return None
        ids: Set[int] = set()
        for tok in re.split(r"[,\s]+", s):
            if not tok:
                continue
            if "-" in tok:
                a, b = tok.split("-", 1)
                lo = int(a, 0); hi = int(b, 0)
                if lo > hi:
                    lo, hi = hi, lo
                ids.update(range(lo, hi + 1))
            else:
                ids.add(int(tok, 0))
        return ids or None

    def _safe_call(self, obj, name, *args, **kwargs):
        fn = getattr(obj, name, None)
        if callable(fn):
            return fn(*args, **kwargs)
        return None

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
        if CanSimpleSender is None or CanSimpleReceiver is None:
            self._err("송신기/수신기 import 실패")
            return

        if self.sender is not None or self._rx_receiver is not None or (self.rxWorker is not None):
            self._append_log("[WARN] 이미 연결되어 있습니다. 먼저 Disconnect 하세요.")
            return

        try:
            # 1) TX: CanSimpleSender 생성 + 연결
            self.sender = CanSimpleSender(
                channel=self.edChannel.text().strip(),
                bitrate_fd=self.edBitrate.text().strip(),
                ifg_us=int(self.edIfg.value()),
                auto_open=False,   # 아래에서 명시적으로 open/connect 호출
            )
            # connect(ifg_us=...) 우선
            try:
                self.sender.connect(ifg_us=int(self.edIfg.value()))
            except TypeError:
                # 구버전 시그니처 대응
                self.sender.connect()
            self._append_log("[INFO] TX 연결 완료")

            # 2) RX: 같은 PCANManager 재사용
            rx_mgr = None
            try:
                rx_mgr = self.sender._require_mgr()
            except Exception:
                rx_mgr = None
            if rx_mgr is None:
                raise RuntimeError("sender._require_mgr() 실패: PCANManager 획득 불가")

            # 3) 수신기 생성 (다중 ID)
            filter_ids = self._parse_filter_ids()
            try:
                self._rx_receiver = CanSimpleReceiver(rx_mgr, filter_can_ids=filter_ids)
            except TypeError:
                one = None if not filter_ids else next(iter(filter_ids))
                self._rx_receiver = CanSimpleReceiver(rx_mgr, filter_can_id=one)

            # 4) 수신 로그 파일 경로 안내
            rx_log = getattr(self._rx_receiver, "_log_path", None)
            if rx_log:
                self._append_log(f"[INFO] 수신 로그 파일: {rx_log}")

            # 5) 수신 콜백 → UI 브릿지를 통해 메인스레드로 전달
            def _on_complete_packet(data: bytes, can_id: int):
                # 수신기 내부에서 hexdump/저장을 이미 수행 → GUI는 표시만
                try:
                    self.uiBus.rx_bin.emit(int(can_id), bytes(data))
                except Exception:
                    pass

            # 6) 콜백 기반 연속 수신 시도 → 실패 시 폴링
            started_callback = False
            try:
                self._rx_receiver.start_rx(on_packet=_on_complete_packet)
                started_callback = True
                self._append_log("[INFO] RX: 콜백 기반 연속 수신 시작(start_rx)")
            except Exception as e:
                self._append_log(f"[WARN] start_rx 실패 → 폴링으로 폴백: {e}")

            if not started_callback:
                try:
                    # 일부 구현은 별도 start_polling이 없으므로 폴링 워커가 직접 호출
                    self.rxWorker = ReceiveWorker(
                        receiver=self._rx_receiver,
                        timeout_s=float(self.edTimeout.value()),
                        verbose=self.chkVerbose.isChecked()
                    )
                    self.rxWorker.rxBinary.connect(lambda data, _id: self.uiBus.rx_bin.emit(int(_id), bytes(data)))
                    self.rxWorker.rxText.connect(lambda text, _id: self.uiBus.rx_txt.emit(int(_id), text))
                    self.rxWorker.error.connect(lambda msg: self.console.append(f"[RX-ERR] {msg}"))
                    self.rxWorker.start()
                    self._append_log("[INFO] RX: 폴링 기반 연속 수신 시작")
                except Exception as e:
                    self._err(f"RX 시작 실패: {e}")
                    self._teardown_on_error()
                    return

            # 7) UI 상태
            self.btnConnect.setEnabled(False)
            self.btnDisconnect.setEnabled(True)
            self.statusBar().showMessage("Connected")
            self._append_log("[INFO] 전체 연결 및 수신 시작 완료")

        except Exception as e:
            self._teardown_on_error()
            self._err(str(e))

    def _teardown_on_error(self):
        try:
            if self.rxWorker is not None:
                self.rxWorker.stop()
                self.rxWorker.wait(500)
                self.rxWorker = None
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
            if self.sender is not None:
                self._safe_call(self.sender, "stop_repl")
                self._safe_call(self.sender, "disconnect")  # 내부에서 PCANManager close
                self.sender = None
        except Exception:
            pass
        self.sender = None

    def on_disconnect(self):
        try:
            if self.rxWorker is not None:
                self.rxWorker.stop()
                self.rxWorker.wait(1000)
                self.rxWorker = None
            if self._rx_receiver is not None:
                try:
                    self._rx_receiver.close()  # 내부적으로 stop_rx + 파일 flush/close
                except Exception:
                    pass
                self._rx_receiver = None
        except Exception:
            pass

        if self.sender is not None:
            self._safe_call(self.sender, "stop_repl")
            self._safe_call(self.sender, "disconnect")
            self.sender = None

        self.btnConnect.setEnabled(True)
        self.btnDisconnect.setEnabled(False)
        self.statusBar().showMessage("Disconnected")
        self._append_log("[INFO] 연결 해제")

    # ================= 전송 =================
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

    # ================= REPL/콘솔 =================
    def _on_repl_rx_text(self, text: str, can_id: int):
        self.console.append(f"[{hex(can_id)}] {text}")

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

    # ================= 메인스레드: 수신 표시 핸들러 =================
    def _on_ui_rx_bin(self, can_id: int, data_obj: object):
        try:
            data = bytes(data_obj) if not isinstance(data_obj, (bytes, bytearray)) else bytes(data_obj)
        except Exception:
            return
        # 콘솔 ASCII hexdump 대신 간단히 HEX 스트링(긴 출력 억제용)
        hexstr = binascii.hexlify(data).decode("ascii").upper()
        if len(hexstr) > 256:
            hexstr = hexstr[:256] + "..."
        self.console.append(f"CAN ID: 0x{can_id:X}\n{hexstr}\n")

    def _on_ui_rx_txt(self, can_id: int, text: str):
        # 텍스트는 그대로 출력
        self.console.append(f"CAN ID: 0x{can_id:X}\n{text}\n")

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
