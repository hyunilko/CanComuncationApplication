#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
can_data_process_gui.py
- PyQt6 GUI for CAN CLI
- TX: can_cli_command_sender.CanCliCommandSender (backend)
- RX: can_data_receiver.CanDataReceiver (프레이밍 조립 후 완성 메시지 수신)

필요 패키지:
    pip install PyQt6
"""

from __future__ import annotations
from typing import List, Optional

import sys, os, binascii
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


# -------- 송신 작업 스레드 --------
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


# -------- 수신 작업 스레드 (CanDataReceiver 사용) --------
class ReceiveWorker(QThread):
    rxText = pyqtSignal(str, int)      # 완성 텍스트, CAN ID (-1은 미상)
    rxBinary = pyqtSignal(bytes, int)  # 완성 바이너리, CAN ID
    error = pyqtSignal(str)

    def __init__(self, receiver: "CanDataReceiver", timeout_s: float, verbose: bool):
        super().__init__()
        self.receiver = receiver
        self.timeout_s = timeout_s
        self.verbose = verbose
        self._stop = False

        # 콜백 기반 연속 수신 지원 여부 감지
        self._has_callback = all(
            hasattr(self.receiver, name) for name in ("start_rx", "stop_rx")
        )

    # ---- 다양한 시그니처 호환: 한 번 수신 시도 ----
    def _recv_once(self):
        """
        가능한 메서드를 순서대로 시도해 한 번 수신.
        성공 시 bytes 또는 (bytes, can_id) 반환, 미수신/타임아웃 시 None.
        """
        # 1) receive_packet(timeout_s=..., verbose=...)
        if hasattr(self.receiver, "receive_packet"):
            try:
                return self.receiver.receive_packet(timeout_s=self.timeout_s, verbose=self.verbose)
            except TypeError:
                # verbose 미지원
                try:
                    return self.receiver.receive_packet(timeout_s=self.timeout_s)
                except TypeError:
                    # 키워드 미지원 → 위치 인자
                    try:
                        return self.receiver.receive_packet(self.timeout_s)
                    except Exception:
                        pass
            except Exception:
                pass

        # 2) receiveData(...) 폴백
        if hasattr(self.receiver, "receiveData"):
            try:
                return self.receiver.receiveData(timeout_s=self.timeout_s)
            except TypeError:
                try:
                    return self.receiver.receiveData(self.timeout_s)
                except Exception:
                    pass
            except Exception:
                pass

        # 수신 API 없음
        raise RuntimeError("Receiver has no compatible receive method (receive_packet/receiveData)")

    # ---- 콜백 핸들러(콜백 기반 연속 수신 모드에서 사용) ----
    def _on_callback_packet(self, pkt):
        """
        pkt: bytes 또는 (bytes, can_id) 가정.
        텍스트/바이너리 자동 분기 후 UI로 신호 전파.
        """
        data, can_id = (pkt, -1) if not isinstance(pkt, tuple) else (pkt[0], pkt[1])
        try:
            text = data.decode("utf-8")
            self.rxText.emit(text, can_id if isinstance(can_id, int) else -1)
        except UnicodeDecodeError:
            self.rxBinary.emit(data, can_id if isinstance(can_id, int) else -1)

    def run(self):
        try:
            if self._has_callback:
                # ----- 이벤트 드리븐 연속 수신 -----
                try:
                    # start_rx가 (callback) 또는 (on_packet=callback) 둘 다 수용하도록 시도
                    try:
                        self.receiver.start_rx(self._on_callback_packet)
                    except TypeError:
                        self.receiver.start_rx(on_packet=self._on_callback_packet)  # type: ignore
                except Exception as e:
                    # 콜백 모드 실패 시 폴링으로 폴백
                    self._has_callback = False
                    self.error.emit(f"start_rx fallback to polling: {e}")

            if self._has_callback:
                # 콜백 모드에서 스레드는 종료 신호만 감시
                while not self._stop:
                    self.msleep(50)
                # 정리
                try:
                    self.receiver.stop_rx()
                except Exception:
                    pass
                try:
                    self.receiver.close()
                except Exception:
                    pass
                return

            # ----- 폴링 연속 수신 -----
            while not self._stop:
                pkt = self._recv_once()
                if pkt is None:
                    # 타임아웃/미수신 → 과도한 CPU 점유 방지
                    self.msleep(2)
                    continue

                # bytes 또는 (bytes, can_id) 모두 처리
                if isinstance(pkt, tuple):
                    data, can_id = pkt[0], pkt[1] if len(pkt) > 1 else -1
                else:
                    data, can_id = pkt, -1

                try:
                    text = data.decode("utf-8")
                    self.rxText.emit(text, can_id if isinstance(can_id, int) else -1)
                except UnicodeDecodeError:
                    self.rxBinary.emit(data, can_id if isinstance(can_id, int) else -1)

        except Exception as e:
            self.error.emit(str(e))

    def stop(self):
        self._stop = True
        # 폴링 모드에서 blocking 되어 있더라도, 다음 루프에서 종료
        # 콜백 모드에선 run() 말미에서 stop_rx/close 호출
        try:
            if hasattr(self.receiver, "wake") and callable(self.receiver.wake):
                # 드물게 블로킹 해제를 위한 wake()가 있을 수 있음
                self.receiver.wake()
        except Exception:
            pass


# -------- REPL 수신 신호 브리지 (백엔드 start_repl 사용 시 대비) --------
class ReplBridge(QObject):
    rxText = pyqtSignal(str, int)  # text, can_id


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("CAN Data Process GUI (PyQt6)")
        self.resize(1280, 860)

        if CanCliCommandSender is None:
            QMessageBox.critical(self, "오류", f"백엔드(can_cli_command_sender) import 실패: {_be_err}")
        if CanDataReceiver is None:
            QMessageBox.critical(self, "오류", f"수신기(can_data_receiver) import 실패: {_rx_err}")

        self.txWorker: Optional[SendWorker] = None
        self.rxWorker: Optional[ReceiveWorker] = None
        self.sender: Optional["CanCliCommandSender"] = None
        self._rx_receiver: Optional[CanDataReceiver] = None

        # 우리가 별도로 연 PCANManager인지(연결 해제 시 닫기 위함)
        self._rx_mgr_owned = False
        self._rx_mgr = None

        # ---------------- 상단: 연결/수신 설정 ----------------
        self.edChannel = QLineEdit("PCAN_USBBUS1")
        self.edBitrate = QLineEdit(
            "f_clock=80000000,nom_brp=2,nom_tseg1=33,nom_tseg2=6,nom_sjw=1,"
            "data_brp=2,data_tseg1=6,data_tseg2=1,data_sjw=1,data_ssp_offset=14"
        )
        self.edIfg = QSpinBox(); self.edIfg.setRange(0, 20000); self.edIfg.setValue(3000)
        self.edFilterId = QLineEdit("0xC0")  # 빈칸이면 전체 수신
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
        row.addWidget(QLabel("Filter ID")); row.addWidget(self.edFilterId)
        row.addSpacing(10)
        row.addWidget(QLabel("Timeout s")); row.addWidget(self.edTimeout)
        row.addWidget(self.chkVerbose)
        row.addStretch(1)
        row.addWidget(self.btnConnect); row.addWidget(self.btnDisconnect)

        gbConn = QGroupBox("Connection (TX: CanCliCommandSender, RX: CanDataReceiver)")
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

        # ---------------- REPL 콘솔 ----------------
        self.console = QTextEdit(); self.console.setReadOnly(True)
        self.console.setPlaceholderText("REPL 콘솔 출력 (수신 메시지: 조립 완료 텍스트/바이너리)")
        self.consoleInput = QLineEdit()
        self.consoleInput.setPlaceholderText("REPL 명령 입력 후 Enter.  ↑/↓: 히스토리,  Ctrl+L: 콘솔 클리어")
        self.hist: List[str] = []; self.hidx: int = 0

        # 백엔드 REPL 대비 브리지(지금은 CanDataReceiver 사용)
        self.replBridge = ReplBridge()
        self.replBridge.rxText.connect(self._on_repl_rx_text)

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
    def _parse_filter_id(self) -> Optional[int]:
        s = self.edFilterId.text().strip()
        if not s:
            return None
        try:
            return int(s, 0)
        except Exception:
            self._append_log(f"[WARN] Filter ID 파싱 실패: {s} (전체 수신)")
            return None

def on_connect(self):
    if CanCliCommandSender is None or CanDataReceiver is None:
        self._err("백엔드/수신기 import 실패")
        return

    # 중복 연결 방지
    if self.sender is not None or self._rx_receiver is not None or (self.rxWorker is not None):
        self._append_log("[WARN] 이미 연결되어 있습니다. 먼저 Disconnect 하세요.")
        return

    try:
        # 1) TX: Sender 인스턴스 생성 및 연결
        self.sender = CanCliCommandSender(
            channel=self.edChannel.text().strip(),
            bitrate_fd=self.edBitrate.text().strip(),
        )
        self.sender.connect(ifg_us=int(self.edIfg.value()))
        self._append_log("[INFO] TX 연결 완료")

        # 2) RX: TX의 PCANManager 재사용 시도 (불가하면 별도 생성)
        rx_mgr = None
        try:
            # private이지만 실사용을 위해 접근 (class 기반 sender)
            rx_mgr = self.sender._require_mgr()  # type: ignore[attr-defined]
        except Exception:
            rx_mgr = None

        if rx_mgr is None:
            # 최후 수단: 별도 매니저 생성/오픈 (환경에 따라 채널 중복 점유 불가할 수 있음)
            if PCANManager is None:
                self._err("PCANManager 로드 실패: RX 매니저 생성 불가")
                # TX도 닫아줌
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

        # 3) 수신기 생성
        self._rx_receiver = CanDataReceiver(rx_mgr, filter_can_id=self._parse_filter_id())

        # 4) 수신 콜백 (완성 메시지 수신 시 즉시 UI 출력)
        def _on_complete_packet(data: bytes, can_id: int):
            try:
                txt = data.decode("utf-8")
                self.console.append(f"[RX 0x{can_id:X}] {txt}")
            except UnicodeDecodeError:
                s = binascii.hexlify(data).decode("ascii")
                if len(s) > 256:
                    s = s[:256] + "..."
                self.console.append(f"[RX 0x{can_id:X}] {len(data)}B: {s}")

        # 5) 콜백 기반 연속 수신 시도 → 실패 시 폴링으로 폴백
        started_callback = False
        try:
            self._rx_receiver.start_rx(on_packet=_on_complete_packet)
            started_callback = True
            self._append_log("[INFO] RX: 콜백 기반 연속 수신 시작(start_rx)")
        except Exception as e:
            self._append_log(f"[WARN] start_rx 실패 → 폴링으로 폴백: {e}")

        if not started_callback:
            # 폴링 스레드 시작(원시 프레임을 큐에 적재)
            try:
                self._rx_receiver.start_polling(poll_interval_s=0.001)
                self._append_log("[INFO] RX: 폴링 기반 연속 수신 시작(start_polling)")
            except Exception as e:
                # 폴링조차 불가 → 정리 후 실패 처리
                self._err(f"RX 시작 실패: {e}")
                try:
                    self.sender.disconnect()
                except Exception:
                    pass
                self.sender = None
                try:
                    if self._rx_mgr_owned and self._rx_mgr is not None and hasattr(self._rx_mgr, 'close'):
                        self._rx_mgr.close()  # type: ignore[attr-defined]
                except Exception:
                    pass
                self._rx_mgr_owned = False
                self._rx_mgr = None
                self._rx_receiver = None
                return

            # 폴링 모드에서는 완성 패킷을 뽑아 UI로 전달하는 워커를 함께 구동
            self.rxWorker = ReceiveWorker(
                receiver=self._rx_receiver,
                timeout_s=float(self.edTimeout.value()),
                verbose=self.chkVerbose.isChecked()
            )
            self.rxWorker.rxText.connect(lambda text, _id: self.console.append(f"[RX] {text}"))
            self.rxWorker.rxBinary.connect(lambda data, _id: (
                self.console.append(f"[RX-BIN] {len(data)} bytes: "
                                    f"{(binascii.hexlify(data).decode('ascii')[:256] + '...') if len(data) > 128 else binascii.hexlify(data).decode('ascii')}")))
            self.rxWorker.error.connect(lambda msg: self.console.append(f"[RX-ERR] {msg}"))
            self.rxWorker.start()

        # 6) UI 상태 업데이트
        self.btnConnect.setEnabled(False)
        self.btnDisconnect.setEnabled(True)
        self.statusBar().showMessage("Connected")
        self._append_log("[INFO] 전체 연결 및 수신 시작 완료")

    except Exception as e:
        # 실패 시 정리
        try:
            if self.rxWorker is not None:
                self.rxWorker.stop()
                self.rxWorker.wait(500)
                self.rxWorker = None
        except Exception:
            pass
        try:
            if self._rx_receiver is not None:
                self._rx_receiver.stop_rx()
                self._rx_receiver.stop_polling()
                self._rx_receiver.close()
                self._rx_receiver = None
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
        self._err(str(e))


    def _on_rx_binary(self, data: bytes, _id: int):
        s = binascii.hexlify(data).decode("ascii")
        if len(s) > 128:
            s = s[:128] + "..."
        self.console.append(f"[RX-BIN] {len(data)} bytes: {s}")

    def on_disconnect(self):
        # RX 먼저 정리
        try:
            if self.rxWorker is not None:
                self.rxWorker.stop()
                self.rxWorker.wait(1000)
                self.rxWorker = None
            if self._rx_receiver is not None:
                try:
                    self._rx_receiver.close()
                except Exception:
                    pass
                self._rx_receiver = None
        except Exception:
            pass

        # 우리가 별도로 연 매니저라면 닫기
        if self._rx_mgr_owned and self._rx_mgr is not None:
            try:
                if hasattr(self._rx_mgr, "close"):
                    self._rx_mgr.close()  # type: ignore[attr-defined]
            except Exception:
                pass
            self._rx_mgr_owned = False
            self._rx_mgr = None

        # TX 정리
        if self.sender is not None:
            try:
                try:
                    self.sender.stop_repl()  # 사용 안 해도 안전
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

    # ================= REPL 콘솔 (TX용 입력창) =================
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
