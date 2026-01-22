# stdlib
import sys
import struct
import time
import socket
import queue
import os
import datetime

# third-party
import numpy as np
import serial
import serial.tools.list_ports


# 嘗試匯入 PyQt5，失敗則提示
try:
    from PyQt5.QtWidgets import (
        QApplication,
        QMainWindow,
        QVBoxLayout,
        QWidget,
        QComboBox,
        QPushButton,
        QLabel,
        QLineEdit,
        QHBoxLayout,
        QCheckBox,
        QDialog,
        QFormLayout,
        QFrame,
        QSizePolicy,
        QGroupBox,
        QScrollArea,
        QFileDialog,
    )
    from PyQt5.QtCore import QThread, pyqtSignal, Qt, QRectF, QPoint, QTimer
    from PyQt5.QtGui import QColor, QFont, QPainter, QPen
    import pyqtgraph as pg
except ImportError as e:
    print(f"CRITICAL ERROR: Missing libraries. {e}")
    sys.exit(1)

# ============================================================
# --- 全域配置參數 ---
# ============================================================

# [關鍵設定] 介面縮放比例
UI_SCALE_FACTOR = 1.5

AIR_SPEED = 343.0
WATER_SPEED = 1500.0
DEFAULT_ENVIRONMENT = "AIR"

BAUD_RATE = 2000000
MAX_ROWS = 300
Y_LABEL_DISTANCE = 50
DEFAULT_LEVELS = (0, 256)

PYTHON_IGNORE_INDEX = 20
INDEX_TOLERANCE = 50

# 動態參數初始值
SPEED_OF_SOUND = AIR_SPEED if DEFAULT_ENVIRONMENT == "AIR" else WATER_SPEED
CURRENT_SAMPLE_DELAY_US = 11.5
SAMPLE_TIME = (CURRENT_SAMPLE_DELAY_US + 3.4) * 1e-6
SAMPLE_RESOLUTION = (SPEED_OF_SOUND * SAMPLE_TIME * 100) / 2

DISPLAY_GAIN = 1.3
DESPECKLE_WINDOW = 3
DESPECKLE_THRESHOLD = 10
SMOOTH_ALPHA = 0.25
TVG_STRENGTH = 1.2

# 渲染優化參數
MAX_DISPLAY_SAMPLES = 2000
FPS_LIMIT = 30

COLOR_MAPS = [
    "thermal",
    "flame",
    "yellowy",
    "bipolar",
    "spectrum",
    "cyclic",
    "greyclip",
    "grey",
    "viridis",
    "plasma",
    "inferno",
    "magma",
]

# Range 選項 (Step)
RANGE_OPTIONS_AIR = [10.0, 5.0, 4.0, 3.0, 2.0, 1.0, 0.5, 0.1]
RANGE_OPTIONS_WATER = [40.0, 20.0, 10.0, 5.0, 4.0, 3.0, 2.0, 1.0]

# 最小總可視深度 (單位：公尺)
MIN_VIEW_METERS_AIR = 0.5
MIN_VIEW_METERS_WATER = 1.8

SPEED_OPTIONS = [
    ("Standard (11.5us)", 11.5),
    ("Long Range (25us)", 25.0),
    ("Ultra Range (50us)", 50.0),
]

# Cycles 選項
CYCLES_OPTIONS = ["8", "16", "32", "64", "128"]

# --- [自動計算] 根據縮放比例計算 UI 尺寸 ---
SIDEBAR_BTN_HEIGHT = int(60 * UI_SCALE_FACTOR)
SIDEBAR_FONT_SIZE = int(15 * UI_SCALE_FACTOR)
ZOOM_FONT_SIZE = int(28 * UI_SCALE_FACTOR)

GAUGE_SIZE = int(40 * UI_SCALE_FACTOR)
GAUGE_FONT_SIZE = int(13 * UI_SCALE_FACTOR)
SETTINGS_FONT_SIZE = int(12 * UI_SCALE_FACTOR)
OVERLAY_FONT_L = int(64 * UI_SCALE_FACTOR)
OVERLAY_FONT_M = int(32 * UI_SCALE_FACTOR)
OVERLAY_FONT_S = int(12 * UI_SCALE_FACTOR)
AXIS_FONT_SIZE = int(10 * UI_SCALE_FACTOR)


# --- 輔助函式 ---
def read_packet(ser):
    if ser.in_waiting == 0:
        return None

    header_bytes = ser.read(9)
    if len(header_bytes) != 9:
        return None
    if header_bytes[0] != 0xAA:
        ser.read(ser.in_waiting)
        return None

    try:
        # [修改] 這裡的解包順序對應 Arduino Header 結構
        # Arduino: start(1), depth(2), drive_freq(2), vDrv(2), num_samples(2)
        # 格式 <BHhHH: Byte, UShort, Short, UShort, UShort
        _, depth, drive_freq, v_drv_scaled, num_samples = struct.unpack(
            "<BHhHH", header_bytes
        )
    except struct.error:
        return None

    if num_samples > 20000 or num_samples < 10:
        ser.read(ser.in_waiting)
        return None

    payload = ser.read(num_samples)
    if len(payload) != num_samples:
        return None

    checksum_bytes = ser.read(1)
    if len(checksum_bytes) != 1:
        return None

    calc_checksum = 0
    for b in header_bytes[1:]:
        calc_checksum ^= b
    for b in payload:
        calc_checksum ^= b

    if calc_checksum != checksum_bytes[0]:
        return None

    values = np.frombuffer(payload, dtype=np.uint8, count=num_samples)
    # 回傳: Raw Data, 深度索引, 驅動頻率, 閾值索引
    return values, min(depth, num_samples), drive_freq, float(v_drv_scaled)


def get_serial_ports():
    ports = [port.device for port in serial.tools.list_ports.comports()]
    return ports if ports else ["No Ports"]


def sonar_display_pipeline_optimized(raw_line, tvg_curve):
    line = raw_line.astype(np.float32) * DISPLAY_GAIN
    np.clip(line, 0, 255, out=line)

    if len(line) > DESPECKLE_WINDOW:
        kernel = np.ones(DESPECKLE_WINDOW) / DESPECKLE_WINDOW
        local_mean = np.convolve(line, kernel, mode="same")
        mask = (line > (local_mean + DESPECKLE_THRESHOLD)) & (
            local_mean < DESPECKLE_THRESHOLD
        )
        line[mask] = 0

    for i in range(1, len(line)):
        line[i] = SMOOTH_ALPHA * line[i] + (1 - SMOOTH_ALPHA) * line[i - 1]

    if len(tvg_curve) == len(line):
        line *= tvg_curve

    return np.clip(line, 0, 255).astype(np.uint8)


# --- [新增] 資料錄製執行緒 ---
class DataRecorder(QThread):
    def __init__(self):
        super().__init__()
        self.queue = queue.Queue()
        self.running = False
        self.file_handle = None
        self.filename = ""
        self.start_time = 0

    def start_recording(self, folder_path="."):
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        self.filename = os.path.join(folder_path, f"sonar_log_{timestamp}.bin")
        try:
            self.file_handle = open(self.filename, "wb")
            self.running = True
            self.start_time = time.time()
            self.start()
            print(f"[Recorder] Started: {self.filename}")
            return True
        except Exception as e:
            print(f"[Recorder] Error opening file: {e}")
            return False

    def stop_recording(self):
        self.running = False
        self.wait()  # 等待 run 迴圈結束
        if self.file_handle:
            self.file_handle.close()
            self.file_handle = None
        print("[Recorder] Stopped.")

    def add_data(self, raw_data, depth_idx, drive_freq, speed_of_sound, delay_us, cycles):
        if self.running:
            # 將所有參數打包放入 Queue
            self.queue.put(
                (
                    time.time(),
                    raw_data,
                    depth_idx,
                    drive_freq, # [新增] 加入頻率
                    speed_of_sound,
                    delay_us,
                    cycles,
                )
            )

    def run(self):
        while self.running or not self.queue.empty():
            try:
                # Get data with timeout to check self.running periodically
                item = self.queue.get(timeout=0.1)
            except queue.Empty:
                continue

            ts, raw_data, depth_idx, freq, sos, delay, cyc = item

            # --- 自訂二進位格式 (Big Endian 或 Little Endian 統一即可) ---
            # [更新] Header 格式 (共 26 bytes)
            # 1. Magic Header (2 bytes): 0xFE, 0xFE
            # 2. Timestamp (8 bytes, double)
            # 3. Depth Index (2 bytes, ushort)
            # 4. Drive Freq (2 bytes, short) - [新增]
            # 5. Speed Of Sound (4 bytes, float)
            # 6. Delay US (4 bytes, float)
            # 7. Cycles (2 bytes, ushort)
            # 8. Data Length (2 bytes, ushort)
            # 9. Raw Data (N bytes)

            try:
                data_len = len(raw_data)
                # Header struct: <2s d H h f f H H (Little Endian)
                # 注意: freq 是 int16 所以用 'h'
                header = struct.pack(
                    "<2s d H h f f H H",
                    b"\xfe\xfe",  # Magic (2)
                    ts,           # Timestamp (8)
                    int(depth_idx), # Depth (2)
                    int(freq),      # Drive Freq (2) - [新增]
                    float(sos),     # SOS (4)
                    float(delay),   # Delay (4)
                    int(cyc),       # Cycles (2)
                    data_len,       # Len (2)
                )

                self.file_handle.write(header)
                self.file_handle.write(raw_data.tobytes())
                self.file_handle.flush()  # 確保寫入磁碟
            except Exception as e:
                print(f"[Recorder] Write Error: {e}")


# --- UI Classes ---
class BasePopup(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowFlags(Qt.Popup | Qt.FramelessWindowHint)
        self.setAttribute(Qt.WA_DeleteOnClose)
        self.font_size = SIDEBAR_FONT_SIZE - int(3 * UI_SCALE_FACTOR)

        self.setStyleSheet(
            f"""
            QWidget {{ background-color: #2b2b2b; border: 1px solid #3e4145; }}
            QPushButton {{ background-color: transparent; color: #e0e0e0; font-family: 'Malgun Gothic'; font-size: {self.font_size}px; font-weight: bold; text-align: center; padding: 8px 5px; border: none; border-bottom: 1px solid #333; }}
            QPushButton:hover {{ background-color: #3e4145; color: white; }}
            QPushButton[active="true"] {{ background-color: #0078d7; color: white; }}
        """
        )


class ColorPopup(BasePopup):
    itemSelected = pyqtSignal(str)

    def __init__(self, current, parent=None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)

        for name in COLOR_MAPS:
            try:
                pg.graphicsItems.GradientEditorItem.Gradients[name]
                btn = QPushButton(name.capitalize())
                if name == current:
                    btn.setProperty("active", True)
                btn.clicked.connect(lambda checked, n=name: self.handle_click(n))
                layout.addWidget(btn)
            except Exception as e:
                print(f"[Error] {e}")
                continue

        self.setFixedWidth(int(140 * UI_SCALE_FACTOR))

    def handle_click(self, name):
        self.itemSelected.emit(name)
        self.close()


class RangePopup(BasePopup):
    itemSelected = pyqtSignal(float)

    def __init__(self, current_val, options, parent=None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)
        try:
            total_depth_m = (parent.current_zoom_samples * SAMPLE_RESOLUTION) / 100.0
            current_step_approx = total_depth_m / 4.0
        except Exception:
            current_step_approx = 0
        for val in options:
            if val < 1.0:
                text = f"{val*100:.0f} cm"
            else:
                text = f"{val:.1f} m" if val % 1 != 0 else f"{val:.0f} m"
            btn = QPushButton(text)
            if abs(val - current_step_approx) < (val * 0.1):
                btn.setProperty("active", True)
            btn.clicked.connect(lambda checked, v=val: self.handle_click(v))
            layout.addWidget(btn)
        self.setFixedWidth(int(110 * UI_SCALE_FACTOR))

    def handle_click(self, val):
        self.itemSelected.emit(val)
        self.close()


class CircularGauge(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedSize(GAUGE_SIZE, GAUGE_SIZE)
        self.value = 1
        self.max_value = 4
        self.bg_color = QColor("#333333")
        self.progress_color = QColor("#00ff00")
        self.setAttribute(Qt.WA_TranslucentBackground)

    def set_value(self, val):
        self.value = val
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        w, h = self.width(), self.height()
        padding = int(4 * UI_SCALE_FACTOR)
        rect = QRectF(padding, padding, w - 2 * padding, h - 2 * padding)

        pen_width = int(3 * UI_SCALE_FACTOR)
        painter.setPen(QPen(self.bg_color, pen_width, Qt.SolidLine, Qt.RoundCap))
        painter.drawArc(rect, 225 * 16, -270 * 16)
        if self.max_value > 0:
            ratio = self.value / self.max_value
            span = -270 * ratio * 16
            painter.setPen(
                QPen(self.progress_color, pen_width, Qt.SolidLine, Qt.RoundCap)
            )
            painter.drawArc(rect, 225 * 16, int(span))

        painter.setPen(Qt.white)
        font = QFont("Malgun Gothic", GAUGE_FONT_SIZE, QFont.Bold)
        painter.setFont(font)
        painter.drawText(rect, Qt.AlignCenter, str(self.value))


class GainGaugeWidget(QFrame):
    clicked = pyqtSignal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setObjectName("gainGaugeWidget")
        self.setFixedHeight(SIDEBAR_BTN_HEIGHT)
        self.setCursor(Qt.PointingHandCursor)
        layout = QHBoxLayout(self)
        layout.setContentsMargins(5, 5, 5, 5)
        layout.setSpacing(5)
        self.gauge = CircularGauge()
        label_layout = QVBoxLayout()
        label_layout.setSpacing(0)
        label_layout.addStretch()
        self.lbl_main = QLabel("LNA")
        self.lbl_main.setObjectName("gainLabel")
        self.lbl_sub = QLabel("Gain")
        self.lbl_sub.setObjectName("gainSubLabel")
        label_layout.addWidget(self.lbl_main)
        label_layout.addWidget(self.lbl_sub)
        label_layout.addStretch()
        layout.addWidget(self.gauge)
        layout.addLayout(label_layout)

    def set_value(self, val):
        self.gauge.set_value(val)

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            self.clicked.emit()


class SerialReader(QThread):
    packet_received = pyqtSignal(object)

    def __init__(self, port, baud_rate):
        super().__init__()
        self.port, self.baud_rate = port, baud_rate
        self.running = True
        self.send_queue = queue.Queue()

    def send_raw_command(self, cmd_bytes):
        self.send_queue.put(cmd_bytes)

    def stop(self):
        self.running = False
        self.quit()
        self.wait()

    def run(self):
        try:
            with serial.Serial(
                self.port, self.baud_rate, timeout=0.1, write_timeout=1
            ) as ser:
                print(f"[Serial] Connected to {self.port} at {self.baud_rate}")
                while self.running:
                    try:
                        while not self.send_queue.empty():
                            cmd = self.send_queue.get_nowait()
                            ser.write(cmd)
                    except Exception as e:
                        print(f"Serial Write Error: {e}")
                    result = read_packet(ser)
                    if result:
                        self.packet_received.emit(result)
                    else:
                        time.sleep(0.001)
        except Exception as e:
            print(f"Serial Error: {e}")


class UDPReader(QThread):
    packet_received = pyqtSignal(object)

    def __init__(self, port):
        super().__init__()
        self.port, self.running = port, True

    def stop(self):
        self.running = False
        self.wait()

    def run(self):
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.settimeout(1.0)
            sock.bind(("", self.port))
            while self.running:
                try:
                    data, _ = sock.recvfrom(65536)
                    pass
                except socket.timeout:
                    continue
                except OSError:
                    continue

        finally:
            sock.close()


# --- SettingsDialog ---
# pylint: disable=attribute-defined-outside-init
class SettingsDialog(QDialog):
    def __init__(self, parent):
        super().__init__(parent)
        self.main_app = parent
        self.echo_thr_val = self.main_app.saved_echo_thr

        self.setWindowTitle("Config")
        self.resize(int(320 * UI_SCALE_FACTOR), int(580 * UI_SCALE_FACTOR))

        self._setup_style()
        self._build_ui()

    # ------------------------------------------------------------
    # UI setup helpers
    # ------------------------------------------------------------

    def _setup_style(self):
        lbl_size = int(11 * UI_SCALE_FACTOR)
        self.setStyleSheet(
            f"""
            QDialog {{ background-color: #2b2b2b; color: #e0e0e0; font-family: 'Malgun Gothic', Arial; }}
            QLabel {{ color: #e0e0e0; font-weight: bold; font-size: {lbl_size}px; }}
            QComboBox, QLineEdit {{ background-color: #3a3a3a; border: 1px solid #555; color: white; padding: 3px; border-radius: 2px; font-size: {lbl_size}px; }}
            QPushButton {{ background-color: #444; border: 1px solid #666; color: white; padding: 6px; border-radius: 3px; font-weight: bold; font-size: {SETTINGS_FONT_SIZE}px; }}
            QPushButton#applyBtn {{ background-color: #0078d7; border-color: #005a9e; }}
            QPushButton#applyBtn:hover {{ background-color: #006cbd; }}
            QGroupBox {{ border: 1px solid #555; border-radius: 4px; margin-top: 10px; padding-top: 5px; font-weight: bold; font-size: {SETTINGS_FONT_SIZE}px; }}
            QGroupBox::title {{ subcontrol-origin: margin; subcontrol-position: top left; left: 7px; padding: 0 2px; background-color: #2b2b2b; color: #00b4ff; }}
            QScrollArea {{ border: none; background-color: transparent; }}
            QWidget#scrollContent {{ background-color: transparent; }}
            """
        )

    def _build_ui(self):
        main_layout = QVBoxLayout(self)
        scroll, scroll_layout = self._create_scroll_area()

        scroll_layout.addWidget(self._build_connection_group())
        scroll_layout.addWidget(self._build_sonar_group())
        scroll_layout.addWidget(self._build_display_group())
        scroll_layout.addWidget(self._build_nmea_group())
        scroll_layout.addWidget(self._build_register_group())
        scroll_layout.addStretch()

        scroll.setWidget(scroll_layout.parent())
        main_layout.addWidget(scroll)
        main_layout.addLayout(self._build_button_bar())

    def _create_scroll_area(self):
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)

        content = QWidget()
        content.setObjectName("scrollContent")
        layout = QVBoxLayout(content)
        layout.setSpacing(int(8 * UI_SCALE_FACTOR))
        layout.setContentsMargins(5, 5, 5, 5)

        return scroll, layout

    # ------------------------------------------------------------
    # Group builders
    # ------------------------------------------------------------

    def _build_connection_group(self):
        group = QGroupBox("CONNECTION")
        layout = QFormLayout(group)

        self.source_combo = QComboBox()
        self.source_combo.addItems(["Serial Port", "UDP Stream"])
        self.source_combo.setCurrentText(self.main_app.connection_source)

        self.serial_combo = QComboBox()
        self.serial_combo.addItems(get_serial_ports())
        self.serial_combo.setCurrentText(self.main_app.serial_port_name)

        self.udp_port_input = QLineEdit(str(self.main_app.udp_port_num))

        self.res_combo = QComboBox()
        self.res_combo.addItems(["2000", "4000", "8000", "12000", "18000"])
        idx = self.res_combo.findText(str(self.main_app.current_max_samples))
        if idx >= 0:
            self.res_combo.setCurrentIndex(idx)

        layout.addRow("Type:", self.source_combo)
        layout.addRow("Port:", self.serial_combo)
        layout.addRow("UDP:", self.udp_port_input)
        layout.addRow("Samples:", self.res_combo)

        self.source_combo.currentTextChanged.connect(self.update_inputs)
        self.update_inputs(self.source_combo.currentText())

        return group

    def _build_sonar_group(self):
        group = QGroupBox("SONAR")
        layout = QFormLayout(group)

        self.speed_dropdown = QComboBox()
        self.speed_dropdown.addItems(
            [f"{AIR_SPEED} m/s (Air)", f"{WATER_SPEED} m/s (Water)"]
        )
        self.speed_dropdown.setCurrentIndex(
            1 if self.main_app.current_speed == WATER_SPEED else 0
        )

        self.delay_combo = QComboBox()
        for label, val in SPEED_OPTIONS:
            self.delay_combo.addItem(label, val)

        for i in range(self.delay_combo.count()):
            if abs(self.delay_combo.itemData(i) - self.main_app.current_sample_delay) < 0.1:
                self.delay_combo.setCurrentIndex(i)
                break

        self.cycles_combo = QComboBox()
        self.cycles_combo.addItems(CYCLES_OPTIONS)
        self.cycles_combo.setCurrentText(str(self.main_app.current_cycles))

        layout.addRow("Env:", self.speed_dropdown)
        layout.addRow("Speed:", self.delay_combo)
        layout.addRow("Cycles:", self.cycles_combo)

        return group

    def _build_display_group(self):
        group = QGroupBox("DISPLAY")
        layout = QFormLayout(group)

        self.large_depth_checkbox = QCheckBox("Show Depth")
        self.large_depth_checkbox.setChecked(self.main_app.large_depth_visible)

        self.overlay_mode_combo = QComboBox()
        self.overlay_mode_combo.addItems(["Auto (Threshold)", "Override (Max)"])
        self.overlay_mode_combo.setCurrentIndex(
            0 if self.main_app.depth_overlay_mode == "Auto" else 1
        )

        self.show_line_checkbox = QCheckBox("Show Depth Profile")
        self.show_line_checkbox.setChecked(self.main_app.show_depth_line)

        self.line_mode_combo = QComboBox()
        self.line_mode_combo.addItems(["Follow Auto", "Follow Override"])
        self.line_mode_combo.setCurrentIndex(
            0 if self.main_app.depth_line_mode == "Auto" else 1
        )

        layout.addRow(self.large_depth_checkbox)
        layout.addRow("Src:", self.overlay_mode_combo)
        layout.addRow(self.show_line_checkbox)
        layout.addRow("Line:", self.line_mode_combo)

        return group

    def _build_nmea_group(self):
        group = QGroupBox("NMEA TCP")
        layout = QFormLayout(group)

        self.nmea_checkbox = QCheckBox("Enable")
        self.nmea_checkbox.setChecked(self.main_app.nmea_output_enabled)

        self.nmea_port_input = QLineEdit(str(self.main_app.nmea_port))

        layout.addRow("On:", self.nmea_checkbox)
        layout.addRow("Port:", self.nmea_port_input)

        return group

    def _build_register_group(self):
        group = QGroupBox("REGISTER")
        layout = QVBoxLayout(group)

        self.lbl_echo_val = QLabel(str(self.echo_thr_val))

        btn_minus = QPushButton("-")
        btn_minus.clicked.connect(self.decrease_echo_thr)

        btn_plus = QPushButton("+")
        btn_plus.clicked.connect(self.increase_echo_thr)

        btn_send = QPushButton("Send")
        btn_send.clicked.connect(self.send_echo_thr_cmd)

        row = QHBoxLayout()
        row.addWidget(QLabel("Echo Thr:"))
        row.addWidget(btn_minus)
        row.addWidget(self.lbl_echo_val)
        row.addWidget(btn_plus)
        row.addStretch()
        row.addWidget(btn_send)

        layout.addLayout(row)
        return group

    def _build_button_bar(self):
        layout = QHBoxLayout()
        apply_btn = QPushButton("Apply")
        apply_btn.setObjectName("applyBtn")
        apply_btn.clicked.connect(self.handle_apply)

        cancel_btn = QPushButton("Cancel")
        cancel_btn.clicked.connect(self.close)

        layout.addStretch()
        layout.addWidget(apply_btn)
        layout.addWidget(cancel_btn)

        return layout

    def decrease_echo_thr(self):
        if self.echo_thr_val > 1:
            self.echo_thr_val -= 1
            self.lbl_echo_val.setText(str(self.echo_thr_val))

    def increase_echo_thr(self):
        if self.echo_thr_val < 16:
            self.echo_thr_val += 1
            self.lbl_echo_val.setText(str(self.echo_thr_val))

    def send_echo_thr_cmd(self):
        if (
            not self.main_app.serial_thread
            or not self.main_app.serial_thread.isRunning()
        ):
            return print("[System] Not connected.")
        data = (self.echo_thr_val - 1) | 0x10
        addr = 0x17
        try:
            self.main_app.serial_thread.send_raw_command(
                struct.pack("BBB", ord("W"), addr, data)
            )
            self.main_app.saved_echo_thr = self.echo_thr_val
            print(f"[System] Echo Thr Sent: {self.echo_thr_val}")
        except Exception as e:
            print(f"[System] Error: {e}")

    def update_inputs(self, text):
        self.serial_combo.setEnabled(text == "Serial Port")
        self.udp_port_input.setEnabled(text != "Serial Port")

    def handle_reg_send(self):
        txt = self.reg_input.text().strip()
        if (
            not self.main_app.serial_thread
            or not self.main_app.serial_thread.isRunning()
        ):
            return print("[System] Not connected.")
        if "," not in txt:
            return print("[System] Format Error")
        try:
            addr, data = [int(x.strip(), 16) for x in txt.split(",")]
            self.main_app.serial_thread.send_raw_command(
                struct.pack("BBB", ord("W"), addr, data)
            )
            print(f"[System] Sent: Addr={hex(addr)}, Data={hex(data)}")
        except Exception as e:
            print(f"[System] Error: {e}")

    def handle_apply(self):
        self.main_app.connection_source = self.source_combo.currentText()
        self.main_app.serial_port_name = self.serial_combo.currentText()
        self.main_app.udp_port_num = (
            int(self.udp_port_input.text())
            if self.udp_port_input.text().isdigit()
            else 5005
        )

        new_samples = int(self.res_combo.currentText())
        if new_samples != self.main_app.current_max_samples:
            self.main_app.change_resolution(new_samples)

        new_delay = self.delay_combo.currentData()
        new_cycles = int(self.cycles_combo.currentText())
        self.main_app.current_cycles = new_cycles

        self.main_app.set_sample_delay(new_delay)

        speed = AIR_SPEED if self.speed_dropdown.currentIndex() == 0 else WATER_SPEED
        if speed != self.main_app.current_speed:
            self.main_app.set_sound_speed(speed)

        self.main_app.large_depth_visible = self.large_depth_checkbox.isChecked()
        self.main_app.depth_overlay.setVisible(self.main_app.large_depth_visible)
        self.main_app.depth_overlay_mode = (
            "Auto" if self.overlay_mode_combo.currentIndex() == 0 else "Override"
        )
        self.main_app.show_depth_line = self.show_line_checkbox.isChecked()
        self.main_app.depth_line_mode = (
            "Auto" if self.line_mode_combo.currentIndex() == 0 else "Override"
        )
        if not self.main_app.show_depth_line:
            self.main_app.depth_line.hide()
        port = (
            int(self.nmea_port_input.text())
            if self.nmea_port_input.text().isdigit()
            else 10110
        )
        self.main_app.configure_nmea_output(self.nmea_checkbox.isChecked(), port)
        self.close()


# --- 主應用程式 ---
class WaterfallApp(QMainWindow):
    def __init__(self):
        super().__init__()
        print("[Init] Starting WaterfallApp...")
        self.recorder = DataRecorder()  # [新增] 初始化錄製器

        self.serial_thread = None
        self.udp_thread = None
        self.connection_source = "Serial Port"
        self.serial_port_name = ""
        ports = get_serial_ports()
        if ports:
            self.serial_port_name = ports[0]
        self.udp_port_num = 5005
        self.is_connected = False
        self.nmea_output_enabled = False
        self.nmea_port = 10110
        self.large_depth_visible = True
        self.depth_overlay_mode = "Auto"
        self.show_depth_line = True
        self.depth_line_mode = "Auto"
        self.current_gradient = "cyclic"

        self.current_speed = SPEED_OF_SOUND
        self.current_max_samples = 2000
        self.current_zoom_samples = self.current_max_samples
        self.lna_gain = 4
        self.saved_echo_thr = 16
        self.min_zoom_samples = 20
        self.current_sample_delay = CURRENT_SAMPLE_DELAY_US

        self.current_cycles = 16

        self.tvg_curve = np.linspace(1.0, TVG_STRENGTH, self.current_max_samples)

        self.latest_frame_data = None
        self.latest_frame_meta = None

        self.setWindowTitle("Open Echo Interface")
        self.resize(int(900 * UI_SCALE_FACTOR), int(550 * UI_SCALE_FACTOR))
        self.setStyleSheet(
            f"""
            * {{ font-family: 'Malgun Gothic', Arial, sans-serif; }}
            QMainWindow {{ background-color: black; }} QWidget {{ background-color: black; color: #e0e0e0; }}
            QFrame#sidebarFrame {{ background-color: #2b2b2b; border-left: 1px solid #1a1a1a; }}
            QPushButton.sidebar_btn {{ background-color: transparent; border: none; border-bottom: 1px solid #3e4145; color: #ccc; font-size: {SIDEBAR_FONT_SIZE}px; font-weight: bold; border-radius: 0px; padding: 10px; }}
            QPushButton.sidebar_btn:hover {{ background-color: #3e4145; color: white; }} QPushButton.sidebar_btn:pressed {{ background-color: #1a1a1a; color: #00aaff; }}
            QPushButton.connect_active {{ background-color: transparent; border: none; border-bottom: 1px solid #3e4145; border-left: 4px solid #ff5555; color: #ff5555; font-size: {SIDEBAR_FONT_SIZE}px; font-weight: bold; padding: 10px; }}
            QPushButton.connect_active:hover {{ background-color: #3e4145; }}
            
            /* [新增] 錄製按鈕樣式 (紅色當作 Recording) */
            QPushButton.record_active {{
                background-color: #aa0000;
                border: none;
                border-bottom: 1px solid #3e4145;
                color: white;
                font-size: {SIDEBAR_FONT_SIZE}px;
                font-weight: bold;
                padding: 10px;
            }}
            QPushButton.record_active:hover {{ background-color: #cc0000; }}

            QFrame#gainGaugeWidget {{ background-color: transparent; border-bottom: 1px solid #3e4145; }} QFrame#gainGaugeWidget:hover {{ background-color: #3e4145; }}
            QLabel#gainLabel {{ background-color: transparent; color: #ccc; font-weight: bold; font-size: {SIDEBAR_FONT_SIZE}px; }}
            QLabel#gainSubLabel {{ background-color: transparent; color: #ccc; font-size: {SIDEBAR_FONT_SIZE - 2}px; margin-top: -2px; }}
            QFrame#footerFrame {{ background-color: #111; border-top: 1px solid #333; }}
            QLabel#footerLabel {{ background-color: transparent; color: #aaa; font-size: {int(10 * UI_SCALE_FACTOR)}px; margin-right: 15px; }}
        """
        )

        self.data = np.zeros((MAX_ROWS, self.current_max_samples))
        self.depth_history = np.full(MAX_ROWS, np.nan)

        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QHBoxLayout(central)
        main_layout.setContentsMargins(0, 0, 0, 0)
        main_layout.setSpacing(0)

        left_container = QWidget()
        left_layout = QVBoxLayout(left_container)
        left_layout.setContentsMargins(0, 0, 0, 0)
        left_layout.setSpacing(0)
        self.waterfall = pg.PlotWidget(background="k")
        vb = self.waterfall.getViewBox()
        vb.setDefaultPadding(0)
        self.waterfall.setMouseEnabled(x=False, y=False)
        self.waterfall.showAxis("right")
        self.waterfall.hideAxis("left")
        self.waterfall.hideAxis("bottom")
        self.waterfall.setMenuEnabled(False)
        self.waterfall.getPlotItem().hideButtons()
        y_right = self.waterfall.getAxis("right")
        y_right.setWidth(int(45 * UI_SCALE_FACTOR))
        y_right.setStyle(showValues=True)
        y_right.setTextPen(pg.mkPen(color="w"))
        y_right.setPen(pg.mkPen(color=(100, 100, 100)))

        self.imageitem = pg.ImageItem(axisOrder="row-major")
        self.waterfall.addItem(self.imageitem)
        self.waterfall.invertY(True)
        self.depth_overlay = pg.TextItem(anchor=(0, 1))
        font = QFont("Malgun Gothic", 12)
        font.setWeight(QFont.DemiBold)
        self.depth_overlay.setFont(font)
        self.depth_overlay.setZValue(200)
        self.waterfall.addItem(self.depth_overlay)
        self.depth_line = pg.PlotCurveItem(
            pen=pg.mkPen(color="k", width=int(6 * UI_SCALE_FACTOR))
        )
        self.depth_line.setZValue(50)
        self.waterfall.addItem(self.depth_line)

        self.set_sound_speed(self.current_speed)

        left_layout.addWidget(self.waterfall)

        footer_frame = QFrame()
        footer_frame.setObjectName("footerFrame")
        footer_layout = QHBoxLayout(footer_frame)
        footer_layout.setContentsMargins(5, 2, 5, 2)
        self.lbl_footer_depth = QLabel("Depth: ---")
        self.lbl_footer_depth.setObjectName("footerLabel")
        self.lbl_footer_ovr = QLabel("Override: ---")
        self.lbl_footer_ovr.setObjectName("footerLabel")
        footer_layout.addWidget(self.lbl_footer_depth)
        footer_layout.addWidget(self.lbl_footer_ovr)
        footer_layout.addStretch()
        left_layout.addWidget(footer_frame)
        main_layout.addWidget(left_container, stretch=1)

        self.colorbar = pg.HistogramLUTWidget()
        self.colorbar.setImageItem(self.imageitem)
        try:
            self.colorbar.item.gradient.loadPreset(self.current_gradient)
        except Exception as e:
            print(f"[Error] {e}")
            pass

        sidebar = QFrame()
        sidebar.setObjectName("sidebarFrame")
        sidebar.setFixedWidth(int(110 * UI_SCALE_FACTOR))
        side_layout = QVBoxLayout(sidebar)
        side_layout.setContentsMargins(0, 0, 0, 0)
        side_layout.setSpacing(0)

        zoom_widget = QWidget()
        zoom_widget.setAttribute(Qt.WA_StyledBackground, True)
        zoom_widget.setStyleSheet("background-color: transparent;")
        zoom_layout = QHBoxLayout(zoom_widget)
        zoom_layout.setContentsMargins(0, 0, 0, 0)
        zoom_layout.setSpacing(0)

        self.btn_plus = QPushButton("+")
        self.btn_plus.setFixedHeight(SIDEBAR_BTN_HEIGHT)
        self.btn_plus.setStyleSheet(
            f"""
            QPushButton {{
                background-color: transparent;
                border: none;
                border-bottom: 1px solid #3e4145;
                border-right: 1px solid #3e4145;
                color: #ccc;
                font-family: 'Malgun Gothic';
                font-size: {ZOOM_FONT_SIZE}px;
                font-weight: bold;
                border-radius: 0px;
            }}
            QPushButton:hover {{ background-color: #3e4145; color: white; }}
            QPushButton:pressed {{ background-color: #1a1a1a; color: #00aaff; }}
        """
        )
        self.btn_plus.clicked.connect(self.zoom_in)

        self.btn_minus = QPushButton("-")
        self.btn_minus.setFixedHeight(SIDEBAR_BTN_HEIGHT)
        self.btn_minus.setStyleSheet(
            f"""
            QPushButton {{
                background-color: transparent;
                border: none;
                border-bottom: 1px solid #3e4145;
                color: #ccc;
                font-family: 'Malgun Gothic';
                font-size: {ZOOM_FONT_SIZE}px;
                font-weight: bold;
                border-radius: 0px;
            }}
            QPushButton:hover {{ background-color: #3e4145; color: white; }}
            QPushButton:pressed {{ background-color: #1a1a1a; color: #00aaff; }}
        """
        )
        self.btn_minus.clicked.connect(self.zoom_out)

        zoom_layout.addWidget(self.btn_plus)
        zoom_layout.addWidget(self.btn_minus)

        side_layout.addWidget(zoom_widget)

        self.btn_range = QPushButton("Range")
        self.btn_range.setProperty("class", "sidebar_btn")
        self.btn_range.setFixedHeight(SIDEBAR_BTN_HEIGHT)
        self.btn_range.clicked.connect(self.show_range_menu)
        side_layout.addWidget(self.btn_range)
        self.btn_color = QPushButton("Color")
        self.btn_color.setProperty("class", "sidebar_btn")
        self.btn_color.setFixedHeight(SIDEBAR_BTN_HEIGHT)
        self.btn_color.clicked.connect(self.show_color_menu)
        side_layout.addWidget(self.btn_color)

        self.lna_widget = GainGaugeWidget()
        self.lna_widget.set_value(self.lna_gain)
        self.lna_widget.clicked.connect(self.cycle_lna_gain)
        side_layout.addWidget(self.lna_widget)

        # Spacer
        spacer = QWidget()
        spacer.setSizePolicy(QSizePolicy.Minimum, QSizePolicy.Expanding)
        spacer.setStyleSheet("background-color: transparent;")
        side_layout.addWidget(spacer)

        # [新增] 錄製按鈕
        self.btn_record = QPushButton("Record")
        self.btn_record.setProperty("class", "sidebar_btn")
        self.btn_record.setFixedHeight(SIDEBAR_BTN_HEIGHT)
        self.btn_record.clicked.connect(self.toggle_recording)
        side_layout.addWidget(self.btn_record)

        self.btn_settings = QPushButton("Settings")
        self.btn_settings.setProperty("class", "sidebar_btn")
        self.btn_settings.setFixedHeight(SIDEBAR_BTN_HEIGHT)
        self.btn_settings.clicked.connect(self.open_settings)
        side_layout.addWidget(self.btn_settings)
        self.btn_connect = QPushButton("Connect")
        self.btn_connect.setProperty("class", "sidebar_btn")
        self.btn_connect.setFixedHeight(SIDEBAR_BTN_HEIGHT)
        self.btn_connect.clicked.connect(self.handle_main_connect)
        side_layout.addWidget(self.btn_connect)
        main_layout.addWidget(sidebar)

        self.update_timer = QTimer()
        self.update_timer.setInterval(1000 // FPS_LIMIT)
        self.update_timer.timeout.connect(self.update_plot_from_buffer)
        self.update_timer.start()

    def change_resolution(self, new_samples):
        print(f"[System] Changing resolution to {new_samples} samples")
        self.current_max_samples = new_samples
        self.data = np.zeros((MAX_ROWS, self.current_max_samples))
        self.depth_history = np.full(MAX_ROWS, np.nan)
        self.depth_line.setData(
            x=np.arange(MAX_ROWS), y=self.depth_history, connect="finite"
        )
        self.tvg_curve = np.linspace(1.0, TVG_STRENGTH, self.current_max_samples)

        if self.serial_thread and self.serial_thread.isRunning():
            cmd = struct.pack(">B H", ord("N"), new_samples)
            self.serial_thread.send_raw_command(cmd)

        self.current_zoom_samples = self.current_max_samples
        self.update_zoom_range()

    def set_sample_delay(self, delay_us):
        global SAMPLE_TIME, SAMPLE_RESOLUTION
        print(
            f"[System] Changing sample delay to {delay_us} us, Cycles to {self.current_cycles}"
        )
        self.current_sample_delay = delay_us
        SAMPLE_TIME = (delay_us + 3.4) * 1e-6
        SAMPLE_RESOLUTION = (self.current_speed * SAMPLE_TIME * 100) / 2

        if self.serial_thread and self.serial_thread.isRunning():
            val = int(delay_us)
            cmd = struct.pack("BBB", ord("D"), val, int(self.current_cycles))
            self.serial_thread.send_raw_command(cmd)
        self.update_zoom_range()

    def set_sound_speed(self, speed):
        global SPEED_OF_SOUND, SAMPLE_RESOLUTION
        SPEED_OF_SOUND = self.current_speed = speed
        SAMPLE_RESOLUTION = (SPEED_OF_SOUND * SAMPLE_TIME * 100) / 2

        ax = self.waterfall.getAxis("right")
        ax.setTickFont(QFont("Arial", AXIS_FONT_SIZE))

        self.update_zoom_range()

    def show_color_menu(self):
        popup = ColorPopup(self.current_gradient, self)
        popup.itemSelected.connect(self.set_gradient)
        popup.adjustSize()
        window_geo = self.geometry()
        popup_width = popup.width()
        popup_height = popup.height()
        center_x = (
            window_geo.x()
            + window_geo.width()
            - int(110 * UI_SCALE_FACTOR)
            - popup_width
        )
        center_y = window_geo.y() + (window_geo.height() - popup_height) // 2
        popup.move(center_x, center_y)
        popup.show()

    def show_range_menu(self):
        raw_options = (
            RANGE_OPTIONS_AIR
            if self.current_speed == AIR_SPEED
            else RANGE_OPTIONS_WATER
        )
        max_phys_depth = (self.current_max_samples * SAMPLE_RESOLUTION) / 100.0

        valid_options = [opt for opt in raw_options if (opt * 4.0) <= max_phys_depth]
        min_depth_limit = (
            MIN_VIEW_METERS_AIR
            if self.current_speed == AIR_SPEED
            else MIN_VIEW_METERS_WATER
        )
        valid_options = [opt for opt in valid_options if (opt * 4.0) >= min_depth_limit]

        if not valid_options:
            valid_options = [raw_options[-1]]

        popup = RangePopup(self.current_zoom_samples, valid_options, self)
        popup.itemSelected.connect(self.set_range_step)

        btn_pos = self.btn_range.mapToGlobal(QPoint(0, 0))
        self.position_popup(popup, btn_pos)

    def position_popup(self, popup, btn_pos):
        popup.adjustSize()
        x = btn_pos.x() - popup.width()
        y = btn_pos.y()
        screen_geo = QApplication.desktop().availableGeometry(btn_pos)
        if y + popup.height() > screen_geo.bottom():
            y = screen_geo.bottom() - popup.height() - 5
        if y < screen_geo.top():
            y = screen_geo.top() + 5
        popup.move(x, y)
        popup.show()

    def set_range_step(self, val):
        total_depth = val * 4.0
        self.current_zoom_samples = (total_depth * 100.0) / SAMPLE_RESOLUTION
        self.update_zoom_range()

    def cycle_lna_gain(self):
        self.lna_gain += 1
        if self.lna_gain > 4:
            self.lna_gain = 1
        self.lna_widget.set_value(self.lna_gain)
        if self.serial_thread and self.serial_thread.isRunning():
            reg_map = {1: 0x05, 2: 0x07, 3: 0x04, 4: 0x06}
            val = reg_map.get(self.lna_gain, 0x06)
            self.serial_thread.send_raw_command(struct.pack("BBB", ord("W"), 0x13, val))

    def zoom_in(self):
        min_total_depth_m = (
            MIN_VIEW_METERS_AIR
            if self.current_speed == AIR_SPEED
            else MIN_VIEW_METERS_WATER
        )
        min_allowed_samples = (min_total_depth_m * 100.0) / SAMPLE_RESOLUTION
        next_samples = self.current_zoom_samples - 200

        if next_samples < min_allowed_samples:
            self.current_zoom_samples = min_allowed_samples
        else:
            self.current_zoom_samples = next_samples

        self.update_zoom_range()

    def zoom_out(self):
        step = 200
        self.current_zoom_samples = min(
            self.current_max_samples, self.current_zoom_samples + step
        )
        self.update_zoom_range()

    def update_zoom_range(self):
        pad_top = self.current_zoom_samples * 0.02
        pad_bottom = self.current_zoom_samples * 0.02
        self.waterfall.setYRange(-pad_top, self.current_zoom_samples + pad_bottom)
        overlay_pos = self.current_zoom_samples - (self.current_zoom_samples * 0.05)
        self.depth_overlay.setPos(10, overlay_pos)

        total_depth_m = (self.current_zoom_samples * SAMPLE_RESOLUTION) / 100.0
        step_m = total_depth_m / 4.0
        tick_depths = [i * step_m for i in range(5)]

        ticks = []
        fmt = "{:.1f}" if step_m < 1 else "{:.1f}"
        for d in tick_depths:
            idx = (d * 100.0) / SAMPLE_RESOLUTION
            ticks.append((idx, fmt.format(d)))

        ax = self.waterfall.getAxis("right")
        ax.setTicks([ticks])
        ax.setTickFont(QFont("Arial", AXIS_FONT_SIZE))

        for item in self.waterfall.items():
            if isinstance(item, pg.InfiniteLine) and item != self.depth_line:
                self.waterfall.removeItem(item)
        for idx, _ in ticks:
            if idx > 0:
                line = pg.InfiniteLine(
                    pos=idx,
                    angle=0,
                    pen=pg.mkPen(color=(150, 150, 150, 150), style=Qt.DashLine),
                )
                self.waterfall.addItem(line)

        color_hex = "#FFFFFF"
        display_val_m = 0.0
        freq_text = f"&nbsp;&nbsp;{40}kHz"
        html_str = f"""<div style="text-align: left; line-height: 90%; font-family: 'Malgun Gothic';"><span style="font-size: {OVERLAY_FONT_L}pt; font-weight: 600; color: {color_hex};">{display_val_m:.1f}</span><span style="font-size: {OVERLAY_FONT_M}pt; font-weight: 600; color: {color_hex};">m</span><br><span style="font-size: {OVERLAY_FONT_S}pt; color: #cccccc; font-weight: 600;">{freq_text}</span></div>"""
        self.depth_overlay.setHtml(html_str)

    def on_packet_received(self, packet):
        self.latest_frame_data = packet[0]
        self.latest_frame_meta = packet[1:]

        # [新增] 傳遞給錄製器
        if self.recorder.running:
            raw, depth, freq, _ = packet
            self.recorder.add_data(
                raw,
                depth,
                freq,
                self.current_speed,
                self.current_sample_delay,
                self.current_cycles,
            )
    # ============================================================
    # --- Helper methods for update_plot_from_buffer (Refactored)
    # ============================================================

    def _sync_buffer_size(self, raw_data):
        """Ensure internal buffers match incoming data size."""
        if len(raw_data) == self.current_max_samples:
            return

        self.current_max_samples = len(raw_data)
        self.data = np.zeros((MAX_ROWS, self.current_max_samples))
        self.tvg_curve = np.linspace(
            1.0, TVG_STRENGTH, self.current_max_samples
        )
        self.depth_history = np.full(MAX_ROWS, np.nan)
        self.depth_line.setData(
            x=np.arange(MAX_ROWS),
            y=self.depth_history,
            connect="finite",
        )

    def _update_waterfall_image(self, filtered_line):
        """Update rolling waterfall image."""
        self.data = np.roll(self.data, -1, axis=0)
        self.data[-1, :] = filtered_line
        self.imageitem.setImage(self.data.T, autoLevels=False)
        self.imageitem.setLevels((10, 220))

    def _compute_depth_value(self, depth_idx, override_idx):
        """Determine reliable depth index and reliability flag."""
        not_blind = depth_idx > PYTHON_IGNORE_INDEX
        diff = abs(int(depth_idx) - int(override_idx))
        is_consistent = diff <= INDEX_TOLERANCE
        is_reliable = not_blind and is_consistent

        target_idx = (
            depth_idx if self.depth_line_mode == "Auto" else override_idx
        )

        return (target_idx if is_reliable else np.nan), is_reliable

    def _update_depth_line(self, depth_value):
        """Update depth history curve."""
        self.depth_history = np.roll(self.depth_history, -1)
        self.depth_history[-1] = depth_value

        if self.show_depth_line:
            self.depth_line.setData(
                x=np.arange(MAX_ROWS),
                y=self.depth_history,
                connect="finite",
            )
            self.depth_line.show()
        else:
            self.depth_line.hide()

    def _update_depth_overlay(
        self,
        depth_idx,
        override_idx,
        drive_frequency,
        is_reliable,
    ):
        """Update overlay text and footer labels."""
        depth_m = (depth_idx * SAMPLE_RESOLUTION) / 100.0
        ovr_m = (override_idx * SAMPLE_RESOLUTION) / 100.0

        self.lbl_footer_depth.setText(f"Depth: {depth_m * 100:.0f} cm")
        self.lbl_footer_ovr.setText(f"Override: {ovr_m * 100:.0f} cm")

        if not self.large_depth_visible:
            return

        color_hex = "#FFFFFF" if is_reliable else "rgba(255, 255, 255, 0.2)"
        display_m = depth_m if is_reliable else 0.0
        freq_text = f"&nbsp;&nbsp;{drive_frequency:.0f}kHz"

        html_str = f"""
        <div style="text-align: left; line-height: 90%; font-family: 'Malgun Gothic';">
            <span style="font-size: {OVERLAY_FONT_L}pt; font-weight: 600; color: {color_hex};">
                {display_m:.1f}
            </span>
            <span style="font-size: {OVERLAY_FONT_M}pt; font-weight: 600; color: {color_hex};">
                m
            </span><br>
            <span style="font-size: {OVERLAY_FONT_S}pt; color: #cccccc; font-weight: 600;">
                {freq_text}
            </span>
        </div>
        """
        self.depth_overlay.setHtml(html_str)

    def update_plot_from_buffer(self):
        if self.latest_frame_data is None:
            return

        raw_data = self.latest_frame_data
        depth_idx, drive_frequency, override_idx = self.latest_frame_meta

        # 1. Sync buffer size if incoming data size changed
        self._sync_buffer_size(raw_data)

        # 2. DSP pipeline
        filtered = sonar_display_pipeline_optimized(
            raw_data, self.tvg_curve
        )

        # 3. Update waterfall image
        self._update_waterfall_image(filtered)

        # 4. Compute depth value and reliability
        depth_value, is_reliable = self._compute_depth_value(
            depth_idx, override_idx
        )

        # 5. Update depth line history
        self._update_depth_line(depth_value)

        # 6. Update overlay and footer
        self._update_depth_overlay(
            depth_idx,
            override_idx,
            drive_frequency,
            is_reliable,
        )

        self.latest_frame_data = None


    # [新增] 處理錄製開關
    def toggle_recording(self):
        if not self.recorder.running:
            # 1. 建立對話框物件 (不再使用靜態函式)
            dialog = QFileDialog(self, "Select Folder to Save Data")
            dialog.setFileMode(QFileDialog.Directory)
            dialog.setOption(QFileDialog.DontUseNativeDialog, True)

            # 2. [關鍵] 強制設定這個對話框的樣式 (傳統白底黑字風格)
            # 這樣可以覆蓋掉主程式的「黑色/透明」設定，確保按鈕與文字絕對清晰
            dialog.setStyleSheet(
                """
                QWidget { 
                    background-color: #f0f0f0; 
                    color: black; 
                    font-family: 'Arial';
                    font-size: 14px;
                }
                QPushButton { 
                    background-color: #e1e1e1; 
                    color: black; 
                    border: 1px solid #adadad; 
                    padding: 6px 12px; 
                    border-radius: 3px;
                    min-width: 60px;
                }
                QPushButton:hover { 
                    background-color: #cce8ff; 
                    border: 1px solid #99d1ff;
                }
                QPushButton:pressed {
                    background-color: #99c9ff;
                }
                QLineEdit { 
                    background-color: white; 
                    color: black; 
                    border: 1px solid #adadad; 
                }
                QListView { 
                    background-color: white; 
                    color: black; 
                    border: 1px solid #adadad;
                }
                QTreeView { 
                    background-color: white; 
                    color: black; 
                    border: 1px solid #adadad;
                }
            """
            )

            # 3. 執行對話框
            if dialog.exec_():
                files = dialog.selectedFiles()
                if files:
                    folder = files[0]
                    if self.recorder.start_recording(folder):
                        self.btn_record.setText("Stop Rec")
                        self.btn_record.setProperty("class", "record_active")
                        self.btn_record.setStyle(self.btn_record.style())
        else:
            self.recorder.stop_recording()
            self.btn_record.setText("Record")
            self.btn_record.setProperty("class", "sidebar_btn")
            self.btn_record.setStyle(self.btn_record.style())

    def handle_main_connect(self):
        if self.is_connected:
            if self.serial_thread:
                self.serial_thread.stop()
                self.serial_thread.wait()  # Ensure clean exit
                self.serial_thread = None
            if self.udp_thread:
                self.udp_thread.stop()
                self.udp_thread.wait()
                self.udp_thread = None
            self.is_connected = False
            self.btn_connect.setText("Connect")
            self.btn_connect.setProperty("class", "sidebar_btn")
            self.btn_connect.setStyle(self.btn_connect.style())
            self.lbl_footer_depth.setText("Depth: ---")
            self.lbl_footer_ovr.setText("Override: ---")
        else:
            if self.connection_source == "Serial Port":
                if not self.serial_port_name or self.serial_port_name == "No Ports":
                    return print("No Serial Port Selected")
                self.serial_thread = SerialReader(self.serial_port_name, BAUD_RATE)
                self.serial_thread.packet_received.connect(self.on_packet_received)
                self.serial_thread.start()

                QThread.msleep(2000)

                cmd = struct.pack(">B H", ord("N"), self.current_max_samples)
                self.serial_thread.send_raw_command(cmd)
                QThread.msleep(50)

                # [關鍵修改] 傳送初始 Delay 與 Cycles
                val = int(self.current_sample_delay)
                cmd = struct.pack("BBB", ord("D"), val, int(self.current_cycles))
                self.serial_thread.send_raw_command(cmd)
                QThread.msleep(50)

                reg_map = {1: 0x05, 2: 0x07, 3: 0x04, 4: 0x06}
                val = reg_map.get(self.lna_gain, 0x06)
                self.serial_thread.send_raw_command(
                    struct.pack("BBB", ord("W"), 0x13, val)
                )

                data = (self.saved_echo_thr - 1) | 0x10
                addr = 0x17
                self.serial_thread.send_raw_command(
                    struct.pack("BBB", ord("W"), addr, data)
                )

            else:
                try:
                    self.udp_thread = UDPReader(self.udp_port_num)
                    self.udp_thread.data_received.connect(self.waterfall_plot_callback)
                    self.udp_thread.start()
                except Exception as e:
                    print(f"[Error] {e}")
                    return
            self.is_connected = True
            self.btn_connect.setText("Stop")
            self.btn_connect.setProperty("class", "connect_active")
            self.btn_connect.setStyle(self.btn_connect.style())

    def open_settings(self):
        dlg = SettingsDialog(self)
        dlg.exec_()

    def set_gradient(self, n):
        self.current_gradient = n
        self.colorbar.item.gradient.loadPreset(n)

    def configure_nmea_output(self, e, p):
        self.nmea_output_enabled, self.nmea_port = e, p

    def closeEvent(self, e):
        # [安全清理]
        if self.recorder.running:
            self.recorder.stop_recording()
        if self.serial_thread:
            self.serial_thread.stop()
            self.serial_thread.wait()  # Wait for thread to finish
        if self.udp_thread:
            self.udp_thread.stop()
            self.udp_thread.wait()
        e.accept()


if __name__ == "__main__":
    os.environ["QT_AUTO_SCREEN_SCALE_FACTOR"] = "1"
    QApplication.setAttribute(Qt.AA_EnableHighDpiScaling)
    QApplication.setAttribute(Qt.AA_UseHighDpiPixmaps)
    app = QApplication(sys.argv)
    font = app.font()
    font.setPointSize(10)
    app.setFont(font)
    window = WaterfallApp()
    window.show()
    sys.exit(app.exec())
