# can_cli_command_sender_gui.py

import sys
from PyQt6.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton,
    QComboBox, QRadioButton, QFileDialog, QTextEdit, QProgressBar, QButtonGroup, QSizePolicy
)
from PyQt6.QtCore import Qt, QThread, pyqtSignal

from can_cli_command_sender import CanDataSender, load_commands_from_file
from pcan_manager import PCANManager

class SendThread(QThread):
    update_status = pyqtSignal(int, str, bool)
    finished = pyqtSignal()

    def __init__(self, sender, commands):
        super().__init__()
        self.sender = sender
        self.commands = commands

    def run(self):
        for idx, cmd in enumerate(self.commands):
            result = self.sender.send_long_command(cmd)
            self.update_status.emit(idx, cmd, result)
            self.msleep(120)
        self.finished.emit()

class SenderGui(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Configuration Selection")
        self.resize(440, 340)

        # CAN 객체
        self.pcan_manager = PCANManager()
        self.pcan_manager.initialize()
        self.sender = CanDataSender(self.pcan_manager)
        self.commands = []
        self.file_path = ""

        layout = QVBoxLayout()
        self.setLayout(layout)

        # -- Title --
        lbl_title = QLabel("<b>Configuration Selection</b>")
        lbl_title.setAlignment(Qt.AlignmentFlag.AlignHCenter)
        layout.addWidget(lbl_title)

        # -- Preset vs Custom --
        h_radio = QHBoxLayout()
        self.radio_preset = QRadioButton("Select Preset Configuration")
        self.radio_custom = QRadioButton("Generate Custom Configuration")
        self.radio_group = QButtonGroup(self)
        self.radio_group.addButton(self.radio_preset)
        self.radio_group.addButton(self.radio_custom)
        self.radio_custom.setChecked(True)
        h_radio.addWidget(self.radio_preset)
        h_radio.addWidget(self.radio_custom)
        layout.addLayout(h_radio)

        # -- Preset ComboBox --
        self.combo_preset = QComboBox()
        self.combo_preset.addItems(["Profile_4Tx4Rx_TDM"])
        self.combo_preset.setEnabled(True)
        layout.addWidget(self.combo_preset)

        # -- Custom Config Upload --
        h_custom = QHBoxLayout()
        self.upload_edit = QLabel("No file selected")
        self.upload_edit.setStyleSheet("border: 1px solid #cccccc; padding: 2px 4px;")
        h_custom.addWidget(self.upload_edit, 3)

        self.upload_btn = QPushButton("↑ Upload")
        self.upload_btn.setEnabled(False)
        self.upload_btn.clicked.connect(self.upload_file)
        h_custom.addWidget(self.upload_btn, 1)
        layout.addLayout(h_custom)

        # -- Send Button --
        self.send_btn = QPushButton("Send Config to Device")
        self.send_btn.setStyleSheet("background-color:#c00; color:white; font-size:16px; font-weight:bold;")
        self.send_btn.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
        self.send_btn.clicked.connect(self.send_config)
        layout.addWidget(self.send_btn)

        # -- Progress/Log --
        self.progress = QProgressBar()
        self.progress.setValue(0)
        layout.addWidget(self.progress)

        self.result_text = QTextEdit()
        self.result_text.setReadOnly(True)
        self.result_text.setFixedHeight(80)
        layout.addWidget(self.result_text)

        # Radio 버튼에 따른 활성/비활성
        self.radio_preset.toggled.connect(self.update_mode)
        self.update_mode()

    def update_mode(self):
        if self.radio_preset.isChecked():
            self.combo_preset.setEnabled(True)
            self.upload_btn.setEnabled(False)
            self.send_btn.setEnabled(True)
            self.upload_edit.setText("No file selected")
            self.commands = [
                "sensorStop 0",
                "channelCfg 153 255 0",
                "chirpComnCfg 8 0 0 256 1 13.1 3",
                "chirpTimingCfg 6 63 0 160 58",
                "adcDataDitherCfg 1",
                "frameCfg 64 0 1358 1 100 0",
                "gpAdcMeasConfig 0 0",
                "guiMonitor 1 1 0 0 0 1",
                "cfarProcCfg 0 2 8 4 3 0 9.0 0",
                "cfarProcCfg 1 2 4 2 2 1 9.0 0",
                "cfarFovCfg 0 0.25 9.0",
                "cfarFovCfg 1 -20.16 20.16",
                "aoaProcCfg 64 64",
                "aoaFovCfg -60 60 -60 60",
                "clutterRemoval 0",
                "factoryCalibCfg 1 0 44 2 0x1ff000",
                "runtimeCalibCfg 1",
                "antGeometryBoard xWRL6844EVM",
                "adcDataSource 0 adc_test_data_0001.bin",
                "adcLogging 0",
                "lowPowerCfg 1",
                "sensorStart 0 0 0 0"
            ]
        else:
            self.combo_preset.setEnabled(False)
            self.upload_btn.setEnabled(True)
            self.send_btn.setEnabled(bool(self.file_path))

    def upload_file(self):
        file_path, _ = QFileDialog.getOpenFileName(
            self,
            "Select Config File",
            "",
            "Config Files (*.cfg);;All Files (*)"
        )
        if file_path:
            self.file_path = file_path
            self.upload_edit.setText(file_path.split("/")[-1])
            try:
                self.commands = load_commands_from_file(file_path)
                self.result_text.clear()
                self.progress.setValue(0)
                self.send_btn.setEnabled(True)
            except Exception as e:
                self.result_text.setText(str(e))
                self.send_btn.setEnabled(False)

    def send_config(self):
        if not self.commands:
            self.result_text.append("No commands loaded.")
            return
        self.send_btn.setEnabled(False)
        self.progress.setValue(0)
        self.result_text.append(f"Start sending {len(self.commands)} commands...\n")
        self.thread = SendThread(self.sender, self.commands)
        self.thread.update_status.connect(self.update_status)
        self.thread.finished.connect(self.send_finished)
        self.thread.start()

    def update_status(self, idx, cmd, success):
        pct = int((idx + 1) / len(self.commands) * 100)
        self.progress.setValue(pct)
        msg = f"[{idx+1}/{len(self.commands)}] {'OK' if success else 'Fail'}: {cmd}"
        self.result_text.append(msg)

    def send_finished(self):
        self.result_text.append("\nAll commands sent.")
        self.send_btn.setEnabled(True)

    def closeEvent(self, event):
        self.pcan_manager.close()
        super().closeEvent(event)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    gui = SenderGui()
    gui.show()
    sys.exit(app.exec())
