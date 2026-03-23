# stdlib
import sys
import struct
import time
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

# 引入系統監控套件
try:
    import psutil
except ImportError:
    psutil = None  # 如果沒安裝，避免程式崩潰

# ============================================================
# --- 全域配置參數 ---
# ============================================================

# [關鍵設定] 介面縮放比例
UI_SCALE_FACTOR = 1.73

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
FPS_LIMIT = 10

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
        # 解包順序對應 Arduino Header 結構
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
    return values, min(depth, num_samples), drive_freq, float(v_drv_scaled)


def get_serial_ports():
    ports = [port.device for port in serial.tools.list_ports.comports()]
    return ports if ports else ["No Ports"]


# --- 資料錄製執行緒 ---
class DataRecorder(QThread):
    def __init__(self):
        super().__init__()
        self.queue = queue.Queue()
        self.running = False
        self.file_handle = None
        self.current_folder = "."
        self.session_base_name = ""
        self.file_sequence = 1
        self.max_file_size_bytes = 500 * 1024 * 1024 

    def start_recording(self, folder_path="."):
        self.current_folder = folder_path
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        self.session_base_name = f"sonar_log_{timestamp}"
        self.file_sequence = 1
        
        if self._open_new_file():
            self.running = True
            self.start()
            return True
        return False

    def _open_new_file(self):
        filename = os.path.join(
            self.current_folder, 
            f"{self.session_base_name}_{self.file_sequence}.bin"
        )
        try:
            if self.file_handle:
                self.file_handle.close()
            self.file_handle = open(filename, "wb")
            print(f"[Recorder] Writing to: {filename}")
            self.file_sequence += 1
            return True
        except Exception as e:
            print(f"[Recorder] Error creating file: {e}")
            return False

    def stop_recording(self):
        self.running = False
        self.wait()
        if self.file_handle:
            self.file_handle.close()
            self.file_handle = None
        print("[Recorder] Stopped.")

    def add_data(self, raw_data, depth_idx, drive_freq, speed_of_sound, delay_us, cycles, cpu, ram, temp, fan):
        if self.running:
            self.queue.put((time.time(), raw_data, depth_idx, drive_freq, speed_of_sound, delay_us, cycles, cpu, ram, temp, fan))

    def run(self):
        while self.running or not self.queue.empty():
            try:
                item = self.queue.get(timeout=0.1)
            except queue.Empty:
                continue

            ts, raw_data, depth_idx, freq, sos, delay, cyc, cpu, ram, temp, fan = item
            try:
                data_len = len(raw_data)
                header = struct.pack(
                    "<2s d H h f f H H f f f I",
                    b"\xfe\xfe", ts, int(depth_idx), int(freq), float(sos), float(delay),
                    int(cyc), data_len, float(cpu), float(ram), float(temp), int(fan)
                )
                self.file_handle.write(header)
                self.file_handle.write(raw_data.tobytes())
                
                if self.file_handle.tell() >= self.max_file_size_bytes:
                    self.file_handle.flush()
                    print(f"[Recorder] File limit reached ({self.max_file_size_bytes/1024/1024:.0f}MB), rotating...")
                    self._open_new_file()
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
            painter.setPen(QPen(self.progress_color, pen_width, Qt.SolidLine, Qt.RoundCap))
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
            with serial.Serial(self.port, self.baud_rate, timeout=0.1, write_timeout=1) as ser:
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
                        time.sleep(0.005)
        except Exception as e:
            print(f"Serial Error: {e}")


# --- SettingsDialog ---
class SettingsDialog(QDialog):
    def __init__(self, parent):
        super().__init__(parent)
        self.main_app = parent
        self.setWindowTitle("Config")
        self.resize(int(340 * UI_SCALE_FACTOR), int(380 * UI_SCALE_FACTOR))
        self.echo_thr_val = self.main_app.saved_echo_thr

        lbl_size = int(11 * UI_SCALE_FACTOR)
        fixed_label_width = int(70 * UI_SCALE_FACTOR) 
        sb_width = int(35 * UI_SCALE_FACTOR)

        self.setStyleSheet(
            f"""
            QDialog {{ background-color: #2b2b2b; color: #e0e0e0; font-family: 'Malgun Gothic', Arial; }}
            QLabel {{ color: #e0e0e0; font-weight: bold; font-size: {lbl_size}px; }}
            QGroupBox QLabel#fieldLabel {{ min-width: {fixed_label_width}px; max-width: {fixed_label_width}px; }}
            QCheckBox {{ font-size: {lbl_size}px; color: #e0e0e0; font-weight: bold; spacing: 5px; }}
            QCheckBox::indicator {{ width: {int(20 * UI_SCALE_FACTOR)}px; height: {int(20 * UI_SCALE_FACTOR)}px; }}
            QComboBox, QLineEdit {{ background-color: #3a3a3a; border: 1px solid #555; color: white; padding: 2px 4px; border-radius: 2px; font-size: {lbl_size}px; min-height: {int(20 * UI_SCALE_FACTOR)}px; }}
            QComboBox::drop-down {{ subcontrol-origin: padding; subcontrol-position: top right; width: {int(20 * UI_SCALE_FACTOR)}px; border-left: 1px solid #555; background-color: #444; }}
            QComboBox::down-arrow {{ width: 0px; height: 0px; border-left: 4px solid transparent; border-right: 4px solid transparent; border-top: 5px solid #ffffff; margin-top: 1px; margin-right: 1px; }}
            QPushButton {{ background-color: #444; border: 1px solid #666; color: white; padding: 4px 8px; border-radius: 3px; font-weight: bold; font-size: {SETTINGS_FONT_SIZE}px; }}
            QPushButton:hover {{ background-color: #555; border-color: #777; }}
            QPushButton#applyBtn {{ background-color: #0078d7; border-color: #005a9e; }}
            QPushButton#applyBtn:hover {{ background-color: #006cbd; }}
            QPushButton#quitBtn {{ background-color: #aa0000; border: 1px solid #ff3333; color: white; }}
            QPushButton#quitBtn:hover {{ background-color: #cc0000; }}
            QGroupBox {{ border: 1px solid #444; border-radius: 4px; margin-top: {int(8 * UI_SCALE_FACTOR)}px; padding-top: {int(12 * UI_SCALE_FACTOR)}px; padding-bottom: 8px; padding-left: 8px; padding-right: 8px; font-weight: bold; font-size: {SETTINGS_FONT_SIZE}px; }}
            QGroupBox::title {{ subcontrol-origin: margin; subcontrol-position: top left; left: 8px; padding: 0 3px; background-color: transparent; color: #00b4ff; }}
            QScrollArea {{ border: none; background-color: transparent; }}
            QWidget#scrollContent {{ background-color: transparent; }}
            QScrollBar:vertical {{ border: none; background: #2b2b2b; width: {sb_width}px; margin: 0px 0px 0px 0px; }}
            QScrollBar::handle:vertical {{ background: #555; min-height: {sb_width}px; border-radius: 4px; }}
            QScrollBar::handle:vertical:pressed {{ background: #0078d7; }}
            QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {{ height: 0px; }}
            QScrollBar::add-page:vertical, QScrollBar::sub-page:vertical {{ background: none; }}
        """
        )

        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(5, 5, 5, 5)
        main_layout.setSpacing(5)
        
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll_content = QWidget()
        scroll_content.setObjectName("scrollContent")
        
        scroll_layout = QVBoxLayout(scroll_content)
        scroll_layout.setSpacing(int(8 * UI_SCALE_FACTOR))
        scroll_layout.setContentsMargins(2, 2, 2, 2)

        # 1. Connection Group
        conn_group = QGroupBox("CONNECTION")
        conn_layout = QFormLayout(conn_group)
        conn_layout.setContentsMargins(5, 5, 5, 5)
        conn_layout.setVerticalSpacing(int(5 * UI_SCALE_FACTOR))
        conn_layout.setLabelAlignment(Qt.AlignLeft)
        
        self.serial_combo = QComboBox()
        self.serial_combo.addItems(get_serial_ports())
        self.serial_combo.setCurrentText(self.main_app.serial_port_name)

        self.res_combo = QComboBox()
        self.res_combo.addItems(["2000", "4000", "8000", "12000", "18000"])
        current_res_str = str(self.main_app.current_max_samples)
        index = self.res_combo.findText(current_res_str)
        if index >= 0:
            self.res_combo.setCurrentIndex(index)

        l1 = QLabel("Port:")
        l1.setObjectName("fieldLabel")
        conn_layout.addRow(l1, self.serial_combo)
        l2 = QLabel("Samples:")
        l2.setObjectName("fieldLabel")
        conn_layout.addRow(l2, self.res_combo)
        scroll_layout.addWidget(conn_group)

        # 2. SONAR Group
        sonar_group = QGroupBox("SONAR")
        sonar_layout = QFormLayout(sonar_group)
        sonar_layout.setContentsMargins(5, 5, 5, 5)
        sonar_layout.setVerticalSpacing(int(5 * UI_SCALE_FACTOR))

        self.speed_dropdown = QComboBox()
        self.speed_dropdown.addItems([f"{AIR_SPEED} m/s (Air)", f"{WATER_SPEED} m/s (Water)"])
        self.speed_dropdown.setCurrentIndex(1 if self.main_app.current_speed == WATER_SPEED else 0)

        self.delay_combo = QComboBox()
        for label, val in SPEED_OPTIONS:
            self.delay_combo.addItem(label, val)
        curr_delay = self.main_app.current_sample_delay
        for i in range(self.delay_combo.count()):
            if abs(self.delay_combo.itemData(i) - curr_delay) < 0.1:
                self.delay_combo.setCurrentIndex(i)
                break

        self.cycles_combo = QComboBox()
        self.cycles_combo.addItems(CYCLES_OPTIONS)
        self.cycles_combo.setCurrentText(str(self.main_app.current_cycles))

        self.blind_input = QLineEdit(str(self.main_app.blind_zone_val))
        
        self.mode_combo = QComboBox()
        self.mode_combo.addItems(["40kHz Only", "200kHz Only", "Dual (40k/200k)"])
        self.mode_combo.setCurrentIndex(self.main_app.operating_mode)
        
        l_env = QLabel("Env:")
        l_env.setObjectName("fieldLabel")
        sonar_layout.addRow(l_env, self.speed_dropdown)
        
        l_speed = QLabel("Speed:")
        l_speed.setObjectName("fieldLabel")
        sonar_layout.addRow(l_speed, self.delay_combo)
        
        l_cyc = QLabel("Cycles:")
        l_cyc.setObjectName("fieldLabel")
        sonar_layout.addRow(l_cyc, self.cycles_combo)
        
        l_blind = QLabel("Blind(cm):")
        l_blind.setObjectName("fieldLabel")
        sonar_layout.addRow(l_blind, self.blind_input)
        
        l_mode = QLabel("Mode:")
        l_mode.setObjectName("fieldLabel")
        sonar_layout.addRow(l_mode, self.mode_combo)
        
        scroll_layout.addWidget(sonar_group)

        # 3. DISPLAY Group (選項已統整)
        disp_group = QGroupBox("DISPLAY")
        disp_layout = QFormLayout(disp_group)
        disp_layout.setContentsMargins(5, 5, 5, 5)
        disp_layout.setVerticalSpacing(int(5 * UI_SCALE_FACTOR))
        
        self.large_depth_checkbox = QCheckBox("Show Depth")
        self.large_depth_checkbox.setChecked(self.main_app.large_depth_visible)
        
        # 統一的三個選項
        combo_items = ["Combination (Auto+Max)", "Threshold Only (Auto)", "Override Only (Max)"]
        
        self.overlay_mode_combo = QComboBox()
        self.overlay_mode_combo.addItems(combo_items)
        self.overlay_mode_combo.setCurrentText(self.main_app.depth_overlay_mode)
        
        self.show_line_checkbox = QCheckBox("Show Depth Profile")
        self.show_line_checkbox.setChecked(self.main_app.show_depth_line)
        
        self.line_mode_combo = QComboBox()
        self.line_mode_combo.addItems(combo_items)
        self.line_mode_combo.setCurrentText(self.main_app.depth_line_mode)

        disp_layout.addRow(self.large_depth_checkbox)
        l_src = QLabel("Src:")
        l_src.setObjectName("fieldLabel")
        disp_layout.addRow(l_src, self.overlay_mode_combo)
        disp_layout.addRow(self.show_line_checkbox)
        l_line = QLabel("Line:")
        l_line.setObjectName("fieldLabel")
        disp_layout.addRow(l_line, self.line_mode_combo)
        scroll_layout.addWidget(disp_group)

        # 4. NMEA Group
        nmea_group = QGroupBox("NMEA TCP")
        nmea_layout = QFormLayout(nmea_group)
        nmea_layout.setContentsMargins(5, 5, 5, 5)
        nmea_layout.setVerticalSpacing(int(5 * UI_SCALE_FACTOR))
        
        self.nmea_checkbox = QCheckBox("Enable")
        self.nmea_checkbox.setChecked(self.main_app.nmea_output_enabled)
        self.nmea_port_input = QLineEdit(str(self.main_app.nmea_port))
        
        l_on = QLabel("On:")
        l_on.setObjectName("fieldLabel")
        nmea_layout.addRow(l_on, self.nmea_checkbox)
        l_port = QLabel("Port:")
        l_port.setObjectName("fieldLabel")
        nmea_layout.addRow(l_port, self.nmea_port_input)
        scroll_layout.addWidget(nmea_group)

        # 5. REGISTER Group
        reg_group = QGroupBox("REGISTER")
        reg_layout = QFormLayout(reg_group) 
        reg_layout.setContentsMargins(5, 5, 5, 5)
        reg_layout.setVerticalSpacing(int(10 * UI_SCALE_FACTOR))
        
        echo_container = QWidget()
        echo_hbox = QHBoxLayout(echo_container)
        echo_hbox.setContentsMargins(0, 0, 0, 0)
        echo_hbox.setSpacing(5)
        self.btn_echo_minus = QPushButton("-")
        self.btn_echo_minus.setFixedSize(int(25 * UI_SCALE_FACTOR), int(20 * UI_SCALE_FACTOR))
        self.btn_echo_minus.clicked.connect(self.decrease_echo_thr)
        self.lbl_echo_val = QLabel(str(self.echo_thr_val))
        self.lbl_echo_val.setAlignment(Qt.AlignCenter)
        self.lbl_echo_val.setFixedWidth(int(25 * UI_SCALE_FACTOR))
        self.lbl_echo_val.setStyleSheet("background-color: transparent; border: none;")
        self.btn_echo_plus = QPushButton("+")
        self.btn_echo_plus.setFixedSize(int(25 * UI_SCALE_FACTOR), int(20 * UI_SCALE_FACTOR))
        self.btn_echo_plus.clicked.connect(self.increase_echo_thr)
        self.btn_echo_send = QPushButton("Send")
        self.btn_echo_send.setCursor(Qt.PointingHandCursor)
        self.btn_echo_send.setFixedWidth(int(50 * UI_SCALE_FACTOR))
        self.btn_echo_send.clicked.connect(self.send_echo_thr_cmd)
        echo_hbox.addWidget(self.btn_echo_minus)
        echo_hbox.addWidget(self.lbl_echo_val)
        echo_hbox.addWidget(self.btn_echo_plus)
        echo_hbox.addStretch() 
        echo_hbox.addWidget(self.btn_echo_send)
        l_thr = QLabel("Echo Thr:")
        l_thr.setObjectName("fieldLabel")
        reg_layout.addRow(l_thr, echo_container)

        raw_container = QWidget()
        raw_hbox = QHBoxLayout(raw_container)
        raw_hbox.setContentsMargins(0, 0, 0, 0)
        raw_hbox.setSpacing(5)
        self.reg_input = QLineEdit()
        self.reg_input.setPlaceholderText("Addr, Data")
        self.reg_btn = QPushButton("Send")
        self.reg_btn.setCursor(Qt.PointingHandCursor)
        self.reg_btn.setFixedWidth(int(50 * UI_SCALE_FACTOR))
        self.reg_btn.clicked.connect(self.handle_reg_send)
        raw_hbox.addWidget(self.reg_input)
        raw_hbox.addWidget(self.reg_btn)
        l_raw = QLabel("Raw Reg:")
        l_raw.setObjectName("fieldLabel")
        reg_layout.addRow(l_raw, raw_container)

        scroll_layout.addWidget(reg_group)
        scroll_layout.addStretch(1) 
        
        quit_btn = QPushButton("QUIT APPLICATION")
        quit_btn.setObjectName("quitBtn")
        quit_btn.setCursor(Qt.PointingHandCursor)
        quit_btn.setFixedHeight(int(35 * UI_SCALE_FACTOR))
        quit_btn.clicked.connect(self.handle_quit_app)
        scroll_layout.addWidget(quit_btn)
        
        scroll.setWidget(scroll_content)
        main_layout.addWidget(scroll)

        btn_layout = QHBoxLayout()
        btn_layout.setContentsMargins(5, 5, 5, 5)
        apply_btn = QPushButton("Apply")
        apply_btn.setObjectName("applyBtn")
        apply_btn.clicked.connect(self.handle_apply)
        cancel_btn = QPushButton("Cancel")
        cancel_btn.clicked.connect(self.close)
        btn_layout.addStretch()
        btn_layout.addWidget(apply_btn)
        btn_layout.addWidget(cancel_btn)
        main_layout.addLayout(btn_layout)

    def decrease_echo_thr(self):
        if self.echo_thr_val > 1:
            self.echo_thr_val -= 1
            self.lbl_echo_val.setText(str(self.echo_thr_val))

    def increase_echo_thr(self):
        if self.echo_thr_val < 16:
            self.echo_thr_val += 1
            self.lbl_echo_val.setText(str(self.echo_thr_val))

    def send_echo_thr_cmd(self):
        if not self.main_app.serial_thread or not self.main_app.serial_thread.isRunning():
            return print("[System] Not connected.")
        data = (self.echo_thr_val - 1) | 0x10
        addr = 0x17
        try:
            self.main_app.serial_thread.send_raw_command(struct.pack("BBB", ord("W"), addr, data))
            self.main_app.saved_echo_thr = self.echo_thr_val
            print(f"[System] Echo Thr Sent: {self.echo_thr_val}")
        except Exception as e:
            print(f"[System] Error: {e}")

    def handle_reg_send(self):
        txt = self.reg_input.text().strip()
        if not self.main_app.serial_thread or not self.main_app.serial_thread.isRunning():
            return print("[System] Not connected.")
        if "," not in txt:
            return print("[System] Format Error")
        try:
            addr, data = [int(x.strip(), 16) for x in txt.split(",")]
            self.main_app.serial_thread.send_raw_command(struct.pack("BBB", ord("W"), addr, data))
            print(f"[System] Sent: Addr={hex(addr)}, Data={hex(data)}")
        except Exception as e:
            print(f"[System] Error: {e}")

    def handle_apply(self):
        self.main_app.serial_port_name = self.serial_combo.currentText()
        new_samples = int(self.res_combo.currentText())
        if new_samples != self.main_app.current_max_samples:
            self.main_app.change_resolution(new_samples)

        new_delay = self.delay_combo.currentData()
        new_cycles = int(self.cycles_combo.currentText())
        self.main_app.current_cycles = new_cycles
        
        try:
            val_cm = int(self.blind_input.text())
            if val_cm < 0: val_cm = 0
            self.main_app.blind_zone_val = val_cm
        except ValueError:
            pass 

        self.main_app.operating_mode = self.mode_combo.currentIndex()
        self.main_app.set_sample_delay(new_delay)

        speed = AIR_SPEED if self.speed_dropdown.currentIndex() == 0 else WATER_SPEED
        if speed != self.main_app.current_speed:
            self.main_app.set_sound_speed(speed)

        self.main_app.large_depth_visible = self.large_depth_checkbox.isChecked()
        self.main_app.depth_overlay.setVisible(self.main_app.large_depth_visible)
        
        # 儲存顯示與測繪的模式設定
        self.main_app.depth_overlay_mode = self.overlay_mode_combo.currentText()
        self.main_app.show_depth_line = self.show_line_checkbox.isChecked()
        self.main_app.depth_line_mode = self.line_mode_combo.currentText()
        
        if not self.main_app.show_depth_line:
            self.main_app.depth_line.hide()
            
        port = int(self.nmea_port_input.text()) if self.nmea_port_input.text().isdigit() else 10110
        self.main_app.configure_nmea_output(self.nmea_checkbox.isChecked(), port)
        self.close()

    def handle_quit_app(self):
        self.close() 
        self.main_app.close() 


# --- 主應用程式 ---
class WaterfallApp(QMainWindow):
    def __init__(self):
        super().__init__()
        print("[Init] Starting WaterfallApp...")
        self.recorder = DataRecorder() 
        self.serial_thread = None
        
        self.serial_port_name = ""
        ports = get_serial_ports()
        if ports:
            self.serial_port_name = ports[0]
        
        self.is_connected = False
        self.nmea_output_enabled = False
        self.nmea_port = 10110
        self.large_depth_visible = True
        
        # 預設為 Combination
        self.depth_overlay_mode = "Combination (Auto+Max)"
        self.show_depth_line = True
        self.depth_line_mode = "Combination (Auto+Max)"
        self.current_gradient = "cyclic"

        self.current_speed = SPEED_OF_SOUND
        self.current_max_samples = 2000
        self.current_zoom_samples = self.current_max_samples
        self.lna_gain = 4
        self.saved_echo_thr = 16
        self.min_zoom_samples = 20
        self.current_sample_delay = CURRENT_SAMPLE_DELAY_US

        self.current_cycles = 16
        self.blind_zone_val = 30
        self.operating_mode = 1 # 0: 40kHz, 1: 200kHz, 2: Dual
        
        # [新增] Colorbar 控制變數
        self.auto_color_enabled = True  # 預設開啟半自動模式
        self.fixed_noise_floor = 10     # 鎖定底噪數值 (0~255)，10 是一個很乾淨的深色背景值

        self.tvg_curve = np.linspace(1.0, TVG_STRENGTH, self.current_max_samples)

        self.latest_frame_data = None
        self.latest_frame_meta = None

        self.current_cpu_usage = 0.0 
        self.current_ram_usage = 0.0
        self.current_temp = 0.0
        self.current_fan_rpm = 0
        self.record_start_time = 0.0

        self.setWindowTitle("Open Echo Interface")
        self.showMaximized()
        self.setStyleSheet(
            f"""
            * {{ font-family: 'Malgun Gothic', Arial, sans-serif; }}
            QMainWindow {{ background-color: black; }} QWidget {{ background-color: black; color: #e0e0e0; }}
            QFrame#sidebarFrame {{ background-color: #2b2b2b; border-left: 1px solid #1a1a1a; }}
            QPushButton.sidebar_btn {{ background-color: transparent; border: none; border-bottom: 1px solid #3e4145; color: #ccc; font-size: {SIDEBAR_FONT_SIZE}px; font-weight: bold; border-radius: 0px; padding: 10px; }}
            QPushButton.sidebar_btn:hover {{ background-color: #3e4145; color: white; }} QPushButton.sidebar_btn:pressed {{ background-color: #1a1a1a; color: #00aaff; }}
            QPushButton.connect_active {{ background-color: transparent; border: none; border-bottom: 1px solid #3e4145; border-left: 4px solid #ff5555; color: #ff5555; font-size: {SIDEBAR_FONT_SIZE}px; font-weight: bold; padding: 10px; }}
            QPushButton.connect_active:hover {{ background-color: #3e4145; }}
            QPushButton.record_active {{ background-color: #aa0000; border: none; border-bottom: 1px solid #3e4145; color: white; font-size: {SIDEBAR_FONT_SIZE}px; font-weight: bold; padding: 10px; }}
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
        
        self.waterfall.setMouseEnabled(x=False, y=False)
        self.waterfall.plotItem.setAutoVisible(y=False)
        self.waterfall.setClipToView(True)
        
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
        self.imageitem.setOpts(autoDownsample=True)
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
        
        self.lbl_cpu = QLabel("CPU: --%")
        self.lbl_cpu.setObjectName("footerLabel") 
        self.lbl_temp = QLabel("Temp: --°C")
        self.lbl_temp.setObjectName("footerLabel")
        self.lbl_fan = QLabel("Fan: -- RPM")
        self.lbl_fan.setObjectName("footerLabel")
        self.lbl_ram = QLabel("RAM: --%")
        self.lbl_ram.setObjectName("footerLabel")

        footer_layout.addWidget(self.lbl_footer_depth) 
        footer_layout.addWidget(self.lbl_footer_ovr)   
        footer_layout.addStretch()  
        footer_layout.addWidget(self.lbl_cpu)  
        footer_layout.addWidget(self.lbl_temp) 
        footer_layout.addWidget(self.lbl_fan)  
        footer_layout.addWidget(self.lbl_ram)  

        left_layout.addWidget(footer_frame)
        main_layout.addWidget(left_container, stretch=1)

        self.stat_timer = QTimer()
        self.stat_timer.setInterval(1000) 
        self.stat_timer.timeout.connect(self.update_system_stats)
        self.stat_timer.start()

        self.colorbar = pg.HistogramLUTWidget()
        self.colorbar.setImageItem(self.imageitem)
        try:
            self.colorbar.item.gradient.loadPreset(self.current_gradient)
        except Exception:
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
            QPushButton {{ background-color: transparent; border: none; border-bottom: 1px solid #3e4145; border-right: 1px solid #3e4145; color: #ccc; font-family: 'Malgun Gothic'; font-size: {ZOOM_FONT_SIZE}px; font-weight: bold; border-radius: 0px; }}
            QPushButton:hover {{ background-color: #3e4145; color: white; }}
            QPushButton:pressed {{ background-color: #1a1a1a; color: #00aaff; }}
        """
        )
        self.btn_plus.clicked.connect(self.zoom_in)

        self.btn_minus = QPushButton("-")
        self.btn_minus.setFixedHeight(SIDEBAR_BTN_HEIGHT)
        self.btn_minus.setStyleSheet(
            f"""
            QPushButton {{ background-color: transparent; border: none; border-bottom: 1px solid #3e4145; color: #ccc; font-family: 'Malgun Gothic'; font-size: {ZOOM_FONT_SIZE}px; font-weight: bold; border-radius: 0px; }}
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
        
        # -------------------------------------------------------------
        # [新增] Auto/Manual Color 切換按鈕
        # -------------------------------------------------------------
        self.btn_auto_color = QPushButton("Color: Auto")
        self.btn_auto_color.setProperty("class", "sidebar_btn")
        self.btn_auto_color.setFixedHeight(SIDEBAR_BTN_HEIGHT)
        self.btn_auto_color.clicked.connect(self.toggle_auto_color)
        side_layout.addWidget(self.btn_auto_color)
        # -------------------------------------------------------------

        self.lna_widget = GainGaugeWidget()
        self.lna_widget.set_value(self.lna_gain)
        self.lna_widget.clicked.connect(self.cycle_lna_gain)
        side_layout.addWidget(self.lna_widget)

        spacer = QWidget()
        spacer.setSizePolicy(QSizePolicy.Minimum, QSizePolicy.Expanding)
        spacer.setStyleSheet("background-color: transparent;")
        side_layout.addWidget(spacer)

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
        print(f"[System] Changing sample delay to {delay_us} us, Cycles to {self.current_cycles}")
        self.current_sample_delay = delay_us
        SAMPLE_TIME = (delay_us + 3.4) * 1e-6
        SAMPLE_RESOLUTION = (self.current_speed * SAMPLE_TIME * 100) / 2

        if self.serial_thread and self.serial_thread.isRunning():
            val = int(delay_us)
            
            calculated_index = int(self.blind_zone_val / SAMPLE_RESOLUTION)
            if calculated_index > 255:
                calculated_index = 255
                print(f"[Warning] Blind zone index clamped to 255 (Requested {self.blind_zone_val}cm is too deep)")
            elif calculated_index < 0:
                calculated_index = 0
                
            print(f"[System] Blind Zone: {self.blind_zone_val} cm -> {calculated_index} points")
            cmd = struct.pack("BBBBB", ord("D"), val, int(self.current_cycles), calculated_index, int(self.operating_mode))
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
        center_x = window_geo.x() + window_geo.width() - int(110 * UI_SCALE_FACTOR) - popup_width
        center_y = window_geo.y() + (window_geo.height() - popup_height) // 2
        popup.move(center_x, center_y)
        popup.show()
        
    # [新增] 處理 Color Mode 切換邏輯
    def toggle_auto_color(self):
        self.auto_color_enabled = not self.auto_color_enabled
        if self.auto_color_enabled:
            self.btn_auto_color.setText("Color: Auto")
            # 恢復一般按鈕樣式
            self.btn_auto_color.setProperty("class", "sidebar_btn")
        else:
            self.btn_auto_color.setText("Color: Manu")
            # 借用 connect_active 的紅色左邊框樣式，提醒使用者現在是手動模式
            self.btn_auto_color.setProperty("class", "connect_active")
            
        # 強制刷新按鈕的 CSS 樣式
        self.btn_auto_color.style().unpolish(self.btn_auto_color)
        self.btn_auto_color.style().polish(self.btn_auto_color)

    def show_range_menu(self):
        raw_options = RANGE_OPTIONS_AIR if self.current_speed == AIR_SPEED else RANGE_OPTIONS_WATER
        max_phys_depth = (self.current_max_samples * SAMPLE_RESOLUTION) / 100.0
        valid_options = [opt for opt in raw_options if (opt * 4.0) <= max_phys_depth]
        min_depth_limit = MIN_VIEW_METERS_AIR if self.current_speed == AIR_SPEED else MIN_VIEW_METERS_WATER
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
        min_total_depth_m = MIN_VIEW_METERS_AIR if self.current_speed == AIR_SPEED else MIN_VIEW_METERS_WATER
        min_allowed_samples = (min_total_depth_m * 100.0) / SAMPLE_RESOLUTION
        
        # [修改] 改用「比例縮放」，每次減少 20% 的視野 (等於放大畫面)
        next_samples = self.current_zoom_samples * 0.8
        
        if next_samples <= min_allowed_samples:
            # 如果已經到達或非常接近極限，就直接 return 不做事
            # 這樣 Y 軸就不會發生無意義的重繪
            if abs(self.current_zoom_samples - min_allowed_samples) < 1.0:
                return 
            self.current_zoom_samples = min_allowed_samples
        else:
            self.current_zoom_samples = next_samples
            
        self.update_zoom_range()

    def zoom_out(self):
        # 避免到達最大值時重複觸發
        if self.current_zoom_samples >= self.current_max_samples:
            if self.current_zoom_samples != self.current_max_samples:
                self.current_zoom_samples = self.current_max_samples
                self.update_zoom_range()
            return
            
        # [修改] 每次擴大 25% 視野 (對應 0.8 的反向操作)
        next_samples = self.current_zoom_samples * 1.25
        
        if next_samples >= self.current_max_samples:
            self.current_zoom_samples = self.current_max_samples
        else:
            self.current_zoom_samples = next_samples
            
        self.update_zoom_range()

    def update_zoom_range(self):
        pad_top = self.current_zoom_samples * 0.02
        pad_bottom = self.current_zoom_samples * 0.02
        self.waterfall.setYRange(-pad_top, self.current_zoom_samples + pad_bottom, padding=0)
        
        overlay_pos = self.current_zoom_samples - (self.current_zoom_samples * 0.05)
        self.depth_overlay.setPos(10, overlay_pos)

        total_depth_m = (self.current_zoom_samples * SAMPLE_RESOLUTION) / 100.0
        
        # -------------------------------------------------------------
        # [新增] Nice Number 演算法：尋找最完美的整數間距 (0.1, 0.2, 0.5, 1, 2...)
        # -------------------------------------------------------------
        raw_step = total_depth_m / 4.0
        if raw_step <= 0:
            raw_step = 0.1
            
        magnitude = 10 ** np.floor(np.log10(raw_step))
        rel = raw_step / magnitude
        
        if rel < 1.5:
            nice_step_m = 1.0 * magnitude
        elif rel < 3.5:
            nice_step_m = 2.0 * magnitude
        elif rel < 7.5:
            nice_step_m = 5.0 * magnitude
        else:
            nice_step_m = 10.0 * magnitude

        # 根據漂亮的間距，生成刻度陣列
        tick_depths = []
        current_d = 0.0
        while current_d <= total_depth_m * 1.01: # 給予 1% 的容差避免漏掉最後一條線
            tick_depths.append(current_d)
            current_d += nice_step_m

        # 動態決定小數點位數 (避免整數時還顯示 .0，或者刻度太小時小數點不夠)
        if nice_step_m >= 1.0:
            fmt = "{:.0f}"
        elif nice_step_m >= 0.1:
            fmt = "{:.1f}"
        else:
            fmt = "{:.2f}"

        # 將公尺換算回點數以供繪圖
        ticks = []
        for d in tick_depths:
            idx = (d * 100.0) / SAMPLE_RESOLUTION
            ticks.append((idx, fmt.format(d)))

        # -------------------------------------------------------------

        ax = self.waterfall.getAxis("right")
        ax.setTicks([ticks])
        ax.setTickFont(QFont("Arial", AXIS_FONT_SIZE))

        # 移除舊的虛線並畫上新的
        for item in self.waterfall.items():
            if isinstance(item, pg.InfiniteLine) and item != self.depth_line:
                self.waterfall.removeItem(item)
                
        for idx, label in ticks:
            # 確保不會畫到畫面外面
            if 0 < idx < self.current_zoom_samples: 
                line = pg.InfiniteLine(
                    pos=idx, angle=0, pen=pg.mkPen(color=(150, 150, 150, 150), style=Qt.DashLine)
                )
                self.waterfall.addItem(line)

        # 確保初始覆疊文字存在
        color_hex = "#FFFFFF"
        display_val_m = 0.0
        freq_text = f"&nbsp;&nbsp;{40}kHz"
        html_str = f"""<div style="text-align: left; line-height: 90%; font-family: 'Malgun Gothic';"><span style="font-size: {OVERLAY_FONT_L}pt; font-weight: 600; color: {color_hex};">{display_val_m:.1f}</span><span style="font-size: {OVERLAY_FONT_M}pt; font-weight: 600; color: {color_hex};">m</span><br><span style="font-size: {OVERLAY_FONT_S}pt; color: #cccccc; font-weight: 600;">{freq_text}</span></div>"""
        self.depth_overlay.setHtml(html_str)

    def on_packet_received(self, packet):
        self.latest_frame_data = packet[0]
        self.latest_frame_meta = packet[1:]

        if self.recorder.running and psutil:
            raw, depth, freq, _ = packet
            self.recorder.add_data(
                raw, depth, freq, self.current_speed, self.current_sample_delay,
                self.current_cycles, self.current_cpu_usage, self.current_ram_usage,
                self.current_temp, self.current_fan_rpm
            )

        self.update_plot_from_buffer()

    def update_plot_from_buffer(self):
        if self.latest_frame_data is None:
            return

        raw_data = self.latest_frame_data
        depth_index, drive_frequency, override_idx = self.latest_frame_meta

        if len(raw_data) != self.current_max_samples:
            if abs(len(raw_data) - self.current_max_samples) > 0:
                self.current_max_samples = len(raw_data)
                self.data = np.zeros((MAX_ROWS, self.current_max_samples))
                self.depth_history = np.full(MAX_ROWS, np.nan)
                self.depth_line.setData(
                    x=np.arange(MAX_ROWS), y=self.depth_history, connect="finite"
                )

        self.data = np.roll(self.data, -1, axis=0)
        self.data[-1, :] = raw_data

        self.imageitem.setImage(self.data.T, autoLevels=False)

        sigma = np.std(self.data)
        mean = np.mean(self.data)
        if sigma == 0:
            sigma = 1
        self.imageitem.setLevels((mean - 2 * sigma, mean + 2 * sigma))


        # -------------------------------------------------------------
        # [修改] 方案一 + 方案三：半自動底噪鎖定 vs 純手動控制
        # -------------------------------------------------------------
        if self.auto_color_enabled:
            sigma = np.std(self.data)
            mean = np.mean(self.data)
            if sigma == 0:
                sigma = 1
                
            # [方案三] 鎖定底噪 (Min Level)，動態峰值 (Max Level)
            min_level = self.fixed_noise_floor 
            max_level = mean + 2.5 * sigma  # 2.5 倍標準差是個很好的強訊號閥值
            
            # 安全保護，避免邏輯錯誤
            if max_level <= min_level:
                max_level = min_level + 1
                
            self.imageitem.setLevels((min_level, max_level))
        
        # 如果 self.auto_color_enabled 是 False (手動模式)
        # 我們就「什麼都不做」！
        # 這樣右側的 HistogramLUTWidget 就可以任由您用滑鼠上下拖曳來改變顏色映射！
        # -------------------------------------------------------------

        depth_m = (depth_index * SAMPLE_RESOLUTION) / 100.0
        ovr_m = (override_idx * SAMPLE_RESOLUTION) / 100.0

        # 基本的盲區過濾
        not_blind = depth_index > PYTHON_IGNORE_INDEX
        diff = abs(int(depth_index) - int(override_idx))
        
        # --- 判斷資料來源與可靠性 ---
        
        # 1. 決定要在「文字」上顯示什麼
        overlay_target_idx = np.nan
        overlay_reliable = False
        
        if self.depth_overlay_mode == "Combination (Auto+Max)":
            if not_blind and diff <= INDEX_TOLERANCE:
                overlay_target_idx = depth_index
                overlay_reliable = True
        elif self.depth_overlay_mode == "Threshold Only (Auto)":
            if not_blind:
                overlay_target_idx = depth_index
                overlay_reliable = True
        elif self.depth_overlay_mode == "Override Only (Max)":
            # Override 通常不受硬體盲區影響，但我們還是加個基本判斷
            if override_idx > PYTHON_IGNORE_INDEX: 
                overlay_target_idx = override_idx
                overlay_reliable = True

        # 2. 決定要在「折線圖」上畫什麼
        line_target_idx = np.nan
        
        if self.depth_line_mode == "Combination (Auto+Max)":
            if not_blind and diff <= INDEX_TOLERANCE:
                line_target_idx = depth_index
        elif self.depth_line_mode == "Threshold Only (Auto)":
            if not_blind:
                line_target_idx = depth_index
        elif self.depth_line_mode == "Override Only (Max)":
            if override_idx > PYTHON_IGNORE_INDEX:
                line_target_idx = override_idx

        # 更新折線圖歷史紀錄
        self.depth_history = np.roll(self.depth_history, -1)
        self.depth_history[-1] = line_target_idx
        
        if self.show_depth_line:
            self.depth_line.setData(
                x=np.arange(MAX_ROWS), y=self.depth_history, connect="finite"
            )
            self.depth_line.show()
        else:
            self.depth_line.hide()

        # 更新大字體顯示
        if self.large_depth_visible:
            color_hex = "#FFFFFF" if overlay_reliable else "rgba(255, 255, 255, 0.2)"
            
            # 如果是 nan，顯示 0.0，不然轉換成公尺
            if np.isnan(overlay_target_idx):
                display_val_m = 0.0
            else:
                display_val_m = (overlay_target_idx * SAMPLE_RESOLUTION) / 100.0
                
            freq_text = f"&nbsp;&nbsp;{drive_frequency:.0f}kHz"
            html_str = f"""<div style="text-align: left; line-height: 90%; font-family: 'Malgun Gothic';"><span style="font-size: {OVERLAY_FONT_L}pt; font-weight: 600; color: {color_hex};">{display_val_m:.1f}</span><span style="font-size: {OVERLAY_FONT_M}pt; font-weight: 600; color: {color_hex};">m</span><br><span style="font-size: {OVERLAY_FONT_S}pt; color: #cccccc; font-weight: 600;">{freq_text}</span></div>"""
            self.depth_overlay.setHtml(html_str)

        # 狀態列永遠顯示原始數據，方便除錯
        self.lbl_footer_depth.setText(f"Depth: {depth_m * 100:.0f} cm")
        self.lbl_footer_ovr.setText(f"Override: {ovr_m * 100:.0f} cm")
        
        self.latest_frame_data = None

    def update_system_stats(self):
        if psutil is None:
            return
        
        try:
            self.current_cpu_usage = psutil.cpu_percent(interval=None)
        except:
            self.current_cpu_usage = 0.0

        try:
            self.current_ram_usage = psutil.virtual_memory().percent
        except:
            self.current_ram_usage = 0.0
            
        try:
            with open("/sys/class/thermal/thermal_zone0/temp", "r") as f:
                self.current_temp = int(f.read()) / 1000.0
        except:
            self.current_temp = 0.0
            
        try:
            with open("/sys/class/hwmon/hwmon2/fan1_input", "r") as f:
                self.current_fan_rpm = int(f.read().strip())
        except:
            self.current_fan_rpm = 0

        if self.recorder.running:
            elapsed_seconds = int(time.time() - self.record_start_time)
            m, s = divmod(elapsed_seconds, 60)
            h, m = divmod(m, 60)
            time_str = f"Rec {h:02d}:{m:02d}:{s:02d}"
            self.btn_record.setText(time_str)

        self.lbl_cpu.setText(f"CPU: {self.current_cpu_usage:.1f}%")
        self.lbl_ram.setText(f"RAM: {self.current_ram_usage:.1f}%")
        self.lbl_temp.setText(f"Temp: {self.current_temp:.1f}°C")
        self.lbl_fan.setText(f"Fan: {self.current_fan_rpm} RPM")

        if self.current_temp > 75:
            f_size = int(10 * UI_SCALE_FACTOR)
            self.lbl_temp.setStyleSheet(f"color: #ff5555; font-weight: bold; font-size: {f_size}px;")
        else:
            self.lbl_temp.setStyleSheet("")

    def toggle_recording(self):
        if not self.recorder.running:
            dialog = QFileDialog(self, "Select Folder to Save Data")
            dialog.setFileMode(QFileDialog.Directory)
            dialog.setOption(QFileDialog.DontUseNativeDialog, True)
            dialog.setStyleSheet(
                """
                QWidget { background-color: #f0f0f0; color: black; font-family: 'Arial'; font-size: 14px; }
                QPushButton { background-color: #e1e1e1; color: black; border: 1px solid #adadad; padding: 6px 12px; border-radius: 3px; min-width: 60px; }
                QPushButton:hover { background-color: #cce8ff; border: 1px solid #99d1ff; }
                QPushButton:pressed { background-color: #99c9ff; }
                QLineEdit { background-color: white; color: black; border: 1px solid #adadad; }
                QListView { background-color: white; color: black; border: 1px solid #adadad; }
                QTreeView { background-color: white; color: black; border: 1px solid #adadad; }
            """
            )
            if dialog.exec_():
                files = dialog.selectedFiles()
                if files:
                    folder = files[0]
                    if self.recorder.start_recording(folder):
                        self.record_start_time = time.time()
                        self.btn_record.setText("00:00:00")
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
                self.serial_thread.wait() 
                self.serial_thread = None
            self.is_connected = False
            self.btn_connect.setText("Connect")
            self.btn_connect.setProperty("class", "sidebar_btn")
            self.btn_connect.setStyle(self.btn_connect.style())
            self.lbl_footer_depth.setText("Depth: ---")
            self.lbl_footer_ovr.setText("Override: ---")
        else:
            if not self.serial_port_name or self.serial_port_name == "No Ports":
                return print("No Serial Port Selected")
            
            self.serial_thread = SerialReader(self.serial_port_name, BAUD_RATE)
            self.serial_thread.packet_received.connect(self.on_packet_received)
            self.serial_thread.start()

            QThread.msleep(2000)

            cmd = struct.pack(">B H", ord("N"), self.current_max_samples)
            self.serial_thread.send_raw_command(cmd)
            QThread.msleep(50)

            val = int(self.current_sample_delay)
            calculated_index = int(self.blind_zone_val / SAMPLE_RESOLUTION)
            if calculated_index > 255: calculated_index = 255
            elif calculated_index < 0: calculated_index = 0
            
            cmd = struct.pack("BBBBB", ord("D"), val, int(self.current_cycles), calculated_index, int(self.operating_mode))
            self.serial_thread.send_raw_command(cmd)
            QThread.msleep(50)

            reg_map = {1: 0x05, 2: 0x07, 3: 0x04, 4: 0x06}
            val = reg_map.get(self.lna_gain, 0x06)
            self.serial_thread.send_raw_command(struct.pack("BBB", ord("W"), 0x13, val))

            data = (self.saved_echo_thr - 1) | 0x10
            addr = 0x17
            self.serial_thread.send_raw_command(struct.pack("BBB", ord("W"), addr, data))

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
        if self.recorder.running:
            self.recorder.stop_recording()
        if self.serial_thread:
            self.serial_thread.stop()
            self.serial_thread.wait()
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
    window.showFullScreen()
    sys.exit(app.exec())
