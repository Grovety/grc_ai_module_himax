import logging

import serial.tools.list_ports
from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtGui import QColor, QImage, QPixmap
from PyQt6.QtWidgets import (
    QCheckBox,
    QComboBox,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QListWidget,
    QListWidgetItem,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QSpinBox,
    QVBoxLayout,
    QWidget,
)

from pygatebox.config import load_config, save_config
from pygatebox.constants import (
    DISTANCE_LABEL,
    INPUT_VALIDATION,
    LAYOUT_STRETCH,
    VIDEO_DISPLAY,
)
from pygatebox.core.device import GateboxDevice
from pygatebox.core.transport import SerialTransport

from .worker import ImageWorker


logger = logging.getLogger(__name__)


class MainWindow(QMainWindow):
    def __init__(self, log_level_override: str | None = None):
        super().__init__()

        # Load configuration
        self.config = load_config()
        self.log_level_override = (str(log_level_override).upper() if log_level_override else None)
        self.effective_log_level = self.log_level_override or self.config["logging"]["level"]
        self.set_logging_level(self.effective_log_level)
        logger.info("Application starting...")

        # Set window geometry from config
        self.setWindowTitle("Gatebox Control Center")
        self.resize(
            self.config["ui"]["window_width"],
            self.config["ui"]["window_height"]
        )

        # Initialize transport with config settings
        self.transport = SerialTransport(
            port=self.config["serial"]["last_port"],
            baudrate=self.config["serial"]["baudrate"],
            timeout=self.config["serial"]["timeout"],
            inter_byte_timeout=self.config["serial"]["inter_byte_timeout"]
        )
        self.device = GateboxDevice(self.transport)
        self.worker = ImageWorker(self.device)
        self.worker.start()

        self.worker.image_received.connect(self.update_frame)
        self.worker.distance_ready.connect(self.update_distance)
        self.worker.history_ready.connect(self.update_history)
        self.worker.permitted_list_ready.connect(self.update_permitted_list)
        self.worker.masks_list_ready.connect(self.update_masks_list)
        self.worker.status_msg.connect(self.update_status)
        self.worker.connection_result.connect(self.on_connection_result)
        self.worker.connection_lost.connect(self.on_connection_lost)

        # Connection timeout timer
        self.connect_timer = QTimer()
        self.connect_timer.setSingleShot(True)
        self.connect_timer.timeout.connect(self.on_connect_timeout)
        self.connect_timeout_ms = 10000  # 10 seconds

        self.init_ui()

    def init_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QHBoxLayout(central)

        # Left column
        left_layout = QVBoxLayout()

        # 1. Connection
        conn_layout = QHBoxLayout()
        self.combo_ports = QComboBox()
        self.refresh_ports()
        btn_refresh = QPushButton("R")
        btn_refresh.setFixedWidth(30)
        btn_refresh.clicked.connect(self.refresh_ports)
        self.btn_connect = QPushButton("Connect")
        self.btn_connect.setCheckable(True)
        self.btn_connect.clicked.connect(self.toggle_connection)

        conn_layout.addWidget(QLabel("Port:"))
        conn_layout.addWidget(self.combo_ports, 1)
        conn_layout.addWidget(btn_refresh)
        conn_layout.addWidget(self.btn_connect)
        left_layout.addLayout(conn_layout)

        # 2. View Options - restore from config
        opts_layout = QHBoxLayout()
        self.chk_image = QCheckBox("Update Camera")
        self.chk_image.setChecked(self.config["features"]["image"])
        self.chk_image.toggled.connect(lambda s: self.toggle_feature("image", s))

        self.chk_history = QCheckBox("Update History")
        self.chk_history.setChecked(self.config["features"]["history"])
        self.chk_history.toggled.connect(lambda s: self.toggle_feature("history", s))

        self.chk_dist = QCheckBox("Update Distance")
        self.chk_dist.setChecked(self.config["features"]["distance"])
        self.chk_dist.toggled.connect(lambda s: self.toggle_feature("distance", s))

        opts_layout.addWidget(self.chk_image)
        opts_layout.addWidget(self.chk_history)
        opts_layout.addWidget(self.chk_dist)
        left_layout.addLayout(opts_layout)

        # 3. Video
        self.lbl_video = QLabel("Disconnected")
        self.lbl_video.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.lbl_video.setStyleSheet(
            f"background-color: {VIDEO_DISPLAY.BACKGROUND_COLOR}; "
            f"color: {VIDEO_DISPLAY.TEXT_COLOR}; "
            f"font-size: {VIDEO_DISPLAY.FONT_SIZE}px;"
        )
        self.lbl_video.setMinimumSize(
            VIDEO_DISPLAY.MIN_WIDTH,
            VIDEO_DISPLAY.MIN_HEIGHT
        )
        left_layout.addWidget(self.lbl_video, 1)

        # 4. Distance
        self.lbl_distance = QLabel("Distance: ---")
        self.lbl_distance.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.lbl_distance.setStyleSheet(
            f"font-size: {DISTANCE_LABEL.FONT_SIZE}px; "
            f"font-weight: {DISTANCE_LABEL.FONT_WEIGHT}; "
            f"color: {DISTANCE_LABEL.COLOR_DEFAULT}; "
            f"border: {DISTANCE_LABEL.BORDER_WIDTH_DEFAULT}px solid {DISTANCE_LABEL.COLOR_DEFAULT}; "
            "padding: 5px;"
        )
        left_layout.addWidget(self.lbl_distance)

        # 5. Status
        self.lbl_status = QLabel("Ready")
        left_layout.addWidget(self.lbl_status)

        main_layout.addLayout(left_layout, LAYOUT_STRETCH.LEFT_PANEL)

        # Right column
        right_layout = QVBoxLayout()

        # 1. History
        grp_history = QGroupBox("History")
        vbox_hist = QVBoxLayout()
        self.list_history = QListWidget()
        self.btn_clear_history = QPushButton("Clear")
        self.btn_clear_history.clicked.connect(self.action_clear_history)
        vbox_hist.addWidget(self.list_history)
        vbox_hist.addWidget(self.btn_clear_history)
        grp_history.setLayout(vbox_hist)
        right_layout.addWidget(grp_history, LAYOUT_STRETCH.right.HISTORY)

        # 2. Permitted
        grp_permitted = QGroupBox("Permitted Plates")
        vbox_perm = QVBoxLayout()
        self.list_permitted = QListWidget()
        perm_input_layout = QHBoxLayout()
        self.txt_plate_input = QLineEdit()
        self.txt_plate_input.setPlaceholderText("Number...")
        self.txt_plate_input.setMaxLength(INPUT_VALIDATION.PLATE_MAX_LEN)
        btn_add_plate = QPushButton("Add")
        btn_add_plate.clicked.connect(self.action_add_plate)
        perm_input_layout.addWidget(self.txt_plate_input)
        perm_input_layout.addWidget(btn_add_plate)
        perm_btns_layout = QHBoxLayout()
        btn_ref_perm = QPushButton("Refresh")
        btn_ref_perm.clicked.connect(self.action_read_permitted)
        btn_del_perm = QPushButton("Delete")
        btn_del_perm.clicked.connect(self.action_delete_selected_plate)
        btn_clr_perm = QPushButton("Clear")
        btn_clr_perm.clicked.connect(self.action_clear_permitted)
        perm_btns_layout.addWidget(btn_ref_perm)
        perm_btns_layout.addWidget(btn_del_perm)
        perm_btns_layout.addWidget(btn_clr_perm)
        vbox_perm.addWidget(self.list_permitted)
        vbox_perm.addLayout(perm_input_layout)
        vbox_perm.addLayout(perm_btns_layout)
        grp_permitted.setLayout(vbox_perm)
        right_layout.addWidget(grp_permitted, LAYOUT_STRETCH.right.PERMITTED)

        # 3. Masks
        grp_masks = QGroupBox("Validation Masks")
        vbox_masks = QVBoxLayout()
        self.list_masks = QListWidget()
        mask_input_layout = QHBoxLayout()
        self.txt_mask_input = QLineEdit()
        self.txt_mask_input.setPlaceholderText("L # C ? <Beijing>...")
        self.txt_mask_input.setToolTip("L-letter, #-digit, C-char/digit, P-province")
        self.txt_mask_input.setMaxLength(INPUT_VALIDATION.MASK_MAX_LEN)
        btn_add_mask = QPushButton("Add")
        btn_add_mask.clicked.connect(self.action_add_mask)
        mask_input_layout.addWidget(self.txt_mask_input)
        mask_input_layout.addWidget(btn_add_mask)
        mask_btns_layout = QHBoxLayout()
        btn_ref_mask = QPushButton("Refresh")
        btn_ref_mask.clicked.connect(self.action_read_masks)
        btn_del_mask = QPushButton("Delete")
        btn_del_mask.clicked.connect(self.action_delete_selected_mask)
        btn_clr_mask = QPushButton("Clear")
        btn_clr_mask.clicked.connect(self.action_clear_masks)
        mask_btns_layout.addWidget(btn_ref_mask)
        mask_btns_layout.addWidget(btn_del_mask)
        mask_btns_layout.addWidget(btn_clr_mask)
        vbox_masks.addWidget(self.list_masks)
        vbox_masks.addLayout(mask_input_layout)
        vbox_masks.addLayout(mask_btns_layout)
        grp_masks.setLayout(vbox_masks)
        right_layout.addWidget(grp_masks, LAYOUT_STRETCH.right.MASKS)

        # 4. Settings
        grp_settings = QGroupBox("Settings")
        settings_layout = QVBoxLayout()

        gate_layout = QHBoxLayout()
        self.spin_gate = QSpinBox()
        self.spin_gate.setRange(10, 3600)
        self.spin_gate.setValue(10)
        btn_set_gate = QPushButton("Set")
        btn_set_gate.clicked.connect(self.action_set_gate)
        gate_layout.addWidget(QLabel("Gate (s):"))
        gate_layout.addWidget(self.spin_gate)
        gate_layout.addWidget(btn_set_gate)

        log_layout = QHBoxLayout()
        self.combo_log_level = QComboBox()
        self.combo_log_level.addItems(["ERROR", "WARNING", "INFO", "DEBUG"])
        self.combo_log_level.setCurrentText(self.effective_log_level)
        self.combo_log_level.currentTextChanged.connect(self.on_log_level_changed)
        log_layout.addWidget(QLabel("Log Level:"))
        log_layout.addWidget(self.combo_log_level)

        settings_layout.addLayout(gate_layout)
        settings_layout.addLayout(log_layout)
        grp_settings.setLayout(settings_layout)
        right_layout.addWidget(grp_settings, LAYOUT_STRETCH.right.SETTINGS)

        main_layout.addLayout(right_layout, LAYOUT_STRETCH.RIGHT_PANEL)

    # --- Feature Toggle Logic ---
    def toggle_feature(self, feature_name, state):
        self.worker.queue_command({
            "action": "toggle_feature",
            "feature": feature_name,
            "state": state
        })

    def refresh_ports(self):
        self.combo_ports.clear()
        ports = serial.tools.list_ports.comports()
        for p in ports:
            self.combo_ports.addItem(p.device)

    # --- Connection Logic ---
    def toggle_connection(self):
        if self.btn_connect.isChecked():
            # Prevent multiple connect attempts
            if not self.btn_connect.isEnabled():
                logger.warning("Connect already in progress, ignoring duplicate click")
                self.btn_connect.setChecked(False)
                return
                
            port = self.combo_ports.currentText()
            if not port:
                self.btn_connect.setChecked(False)
                return
            self.lbl_status.setText(f"Connecting to {port}...")
            self.btn_connect.setEnabled(False)
            # Start connection timeout timer
            self.connect_timer.start(self.connect_timeout_ms)
            self.worker.queue_command({"action": "connect", "port": port})
        else:
            # Prevent disconnect if not connected
            if self.btn_connect.text() == "Connect":
                logger.warning("Disconnect clicked but not connected")
                self.btn_connect.setChecked(False)
                return
                
            self.worker.queue_command({"action": "disconnect"})
            self.clear_runtime_lists()
            self.lbl_video.setText("Disconnected")
            self.lbl_video.clear()
            self.lbl_distance.setText("Distance: ---")
            self.lbl_distance.setStyleSheet("font-size: 18px; font-weight: bold; color: gray; border: 1px solid gray; padding: 5px;")
            self.btn_connect.setText("Connect")
            self.btn_connect.setStyleSheet("")

    def on_connection_result(self, success: bool, msg: str):
        # Stop connection timeout timer
        self.connect_timer.stop()
        
        self.btn_connect.setEnabled(True)
        self.lbl_status.setText(msg)
        if success:
            self.btn_connect.setText("Disconnect")
            self.btn_connect.setStyleSheet("background-color: #ffcccc")
            self.btn_connect.setChecked(True)
            self.action_read_permitted()
            self.action_read_masks()
        else:
            self.btn_connect.setChecked(False)
            self.btn_connect.setText("Connect")
            self.lbl_video.clear()
            self.lbl_video.setText("Disconnected")
            if msg in ("Disconnected", "Connection lost"):
                self.clear_runtime_lists()
            if "Disconnected" not in msg:
                QMessageBox.critical(self, "Connection Error", msg)

    def on_connect_timeout(self):
        """Handle connection timeout."""
        logger.warning("Connection timeout - resetting button state")
        self.btn_connect.setEnabled(True)
        self.btn_connect.setChecked(False)
        self.btn_connect.setText("Connect")
        self.btn_connect.setStyleSheet("")
        self.lbl_status.setText("Connection timeout - check device and try again")
        QMessageBox.warning(
            self,
            "Connection Timeout",
            "Failed to connect within 10 seconds.\n\n"
            "Please check:\n"
            "• Device is powered on\n"
            "• Correct COM port is selected\n"
            "• Cable is properly connected"
        )

    def on_connection_lost(self):
        """Handle unexpected connection loss."""
        logger.info("Connection lost, updating UI")
        self.clear_runtime_lists()
        self.btn_connect.setEnabled(True)
        self.btn_connect.setChecked(False)
        self.btn_connect.setText("Connect")
        self.btn_connect.setStyleSheet("")
        self.lbl_video.setText("Connection lost")
        self.lbl_video.clear()
        self.lbl_distance.setText("Distance: ---")
        self.lbl_distance.setStyleSheet(
            f"font-size: {DISTANCE_LABEL.FONT_SIZE}px; "
            f"font-weight: {DISTANCE_LABEL.FONT_WEIGHT}; "
            f"color: {DISTANCE_LABEL.COLOR_DEFAULT}; "
            f"border: {DISTANCE_LABEL.BORDER_WIDTH_DEFAULT}px solid {DISTANCE_LABEL.COLOR_DEFAULT}; "
            "padding: 5px;"
        )
        # Refresh ports list
        self.refresh_ports()
        self.lbl_status.setText("Connection lost - select port and connect")

    # --- UI Updates from Signals ---
    def update_frame(self, data: bytes):
        image = QImage.fromData(data)
        if not image.isNull():
            pix = QPixmap.fromImage(image)
            scaled = pix.scaled(self.lbl_video.size(), Qt.AspectRatioMode.KeepAspectRatio)
            self.lbl_video.setPixmap(scaled)

    def update_distance(self, status: int, dist: int):
        if status == 0:
            self.lbl_distance.setText(f"Distance: {dist} mm")
            self.lbl_distance.setStyleSheet("font-size: 18px; font-weight: bold; color: green; border: 2px solid green;")
        else:
            self.lbl_distance.setText(f"Distance: Error ({status})")
            self.lbl_distance.setStyleSheet("font-size: 18px; font-weight: bold; color: red; border: 1px solid red;")

    def update_status(self, msg: str):
        self.lbl_status.setText(msg)

    def update_history(self, history_items: list):
        self.list_history.clear()
        recent_items = history_items[-10:]
        for item in reversed(recent_items):
            parts = item.split(',')
            if len(parts) >= 3:
                display_str = f"{parts[0]} | {parts[1]} | {parts[2]}"
                list_item = QListWidgetItem(display_str)
                if 'Y' in parts[2]:
                    list_item.setForeground(QColor("green"))
                    font = list_item.font()
                    font.setBold(True)
                    list_item.setFont(font)
                else:
                    list_item.setForeground(QColor("black"))
                self.list_history.addItem(list_item)

    def update_permitted_list(self, items: list):
        self.list_permitted.clear()
        for item in items:
            self.list_permitted.addItem(item)

    def update_masks_list(self, items: list):
        self.list_masks.clear()
        for item in items:
            self.list_masks.addItem(item)

    def set_logging_level(self, level_name: str):
        normalized = str(level_name).upper()
        level = getattr(logging, normalized, logging.INFO)
        logging.getLogger().setLevel(level)
        self.config["logging"]["level"] = normalized

    def on_log_level_changed(self, level_name: str):
        self.set_logging_level(level_name)
        self.lbl_status.setText(f"Log level: {level_name}")

    def clear_runtime_lists(self):
        self.list_history.clear()
        self.list_permitted.clear()
        self.list_masks.clear()

    # --- Action Commands (Async) ---
    def action_read_permitted(self):
        self.worker.queue_command({"action": "read_permitted"})

    def action_add_plate(self):
        text = self.txt_plate_input.text().strip()
        if not text:
            self.lbl_status.setText("Plate is empty")
            return
        if len(text) > INPUT_VALIDATION.PLATE_MAX_LEN:
            self.lbl_status.setText(f"Plate is too long (max {INPUT_VALIDATION.PLATE_MAX_LEN})")
            return

        forbidden = INPUT_VALIDATION.FORBIDDEN_COMMON + INPUT_VALIDATION.FORBIDDEN_PLATE_EXTRA
        if any(ch in text for ch in forbidden):
            self.lbl_status.setText("Plate contains forbidden delimiters")
            return

        self.worker.queue_command({"action": "add_plate", "plate": text})
        self.txt_plate_input.clear()

    def action_clear_permitted(self):
        self.worker.queue_command({"action": "clear_permitted"})

    def action_delete_selected_plate(self):
        item = self.list_permitted.currentItem()
        if item is None:
            self.lbl_status.setText("Select a plate to delete")
            return

        plate = item.text().strip()
        if plate:
            self.worker.queue_command({"action": "delete_plate", "plate": plate})

    def action_clear_history(self):
        self.worker.queue_command({"action": "clear_history"})

    def action_set_gate(self):
        self.worker.queue_command({"action": "set_gate", "value": self.spin_gate.value()})

    def action_read_masks(self):
        self.worker.queue_command({"action": "read_masks"})

    def action_add_mask(self):
        text = self.txt_mask_input.text().strip()
        if not text:
            self.lbl_status.setText("Mask is empty")
            return
        if len(text) > INPUT_VALIDATION.MASK_MAX_LEN:
            self.lbl_status.setText(f"Mask is too long (max {INPUT_VALIDATION.MASK_MAX_LEN})")
            return

        if any(ch in text for ch in INPUT_VALIDATION.FORBIDDEN_COMMON):
            self.lbl_status.setText("Mask contains forbidden delimiters")
            return

        self.worker.queue_command({"action": "add_mask", "mask": text})
        self.txt_mask_input.clear()

    def action_delete_selected_mask(self):
        item = self.list_masks.currentItem()
        if item is None:
            self.lbl_status.setText("Select a mask to delete")
            return

        mask = item.text().strip()
        if mask:
            self.worker.queue_command({"action": "delete_mask", "mask": mask})

    def action_clear_masks(self):
        self.worker.queue_command({"action": "clear_masks"})

    def closeEvent(self, event):
        """Save configuration and cleanup on application close."""
        logger.info("Application closing, saving configuration...")

        # Save current state to config
        self.config["serial"]["last_port"] = self.combo_ports.currentText()
        self.config["ui"]["window_width"] = self.width()
        self.config["ui"]["window_height"] = self.height()
        self.config["features"]["image"] = self.chk_image.isChecked()
        self.config["features"]["history"] = self.chk_history.isChecked()
        self.config["features"]["distance"] = self.chk_dist.isChecked()
        self.config["logging"]["level"] = self.combo_log_level.currentText()

        save_config(self.config)
        logger.info("Configuration saved")

        # Stop worker
        self.worker.stop()
        logger.info("Worker stopped")
        event.accept()
