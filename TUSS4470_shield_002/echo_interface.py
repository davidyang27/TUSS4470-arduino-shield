import sys
import numpy as np
import serial
import serial.tools.list_ports
import struct
import time
import socket
import queue
import os
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QVBoxLayout, QWidget, QComboBox, 
    QPushButton, QLabel, QLineEdit, QHBoxLayout, QCheckBox, 
    QDialog, QFormLayout, QFrame, QSizePolicy, QGroupBox, QScrollArea
)
from PyQt5.QtCore import QThread, pyqtSignal, Qt, QSize, QRectF, QPoint
from PyQt5.QtGui import QPalette, QColor, QFont, QPainter, QPen, QBrush

import pyqtgraph as pg

# ============================================================
# --- 全域配置參數 ---
# ============================================================
AIR_SPEED = 343.0      
WATER_SPEED = 1440.0   
DEFAULT_ENVIRONMENT = 'AIR' 

BAUD_RATE = 2000000 
MAX_ROWS = 300
Y_LABEL_DISTANCE = 50  
SAMPLE_TIME = 13.2e-6
DEFAULT_LEVELS = (0, 256)

PYTHON_IGNORE_INDEX = 20
INDEX_TOLERANCE = 50

SPEED_OF_SOUND = AIR_SPEED if DEFAULT_ENVIRONMENT == 'AIR' else WATER_SPEED 
SAMPLE_RESOLUTION = (SPEED_OF_SOUND * SAMPLE_TIME * 100) / 2

DISPLAY_GAIN = 1.3
DESPECKLE_WINDOW = 3
DESPECKLE_THRESHOLD = 10
SMOOTH_ALPHA = 0.25
TVG_STRENGTH = 1.2

SIDEBAR_BTN_FONT_SIZE = 15
COLOR_MAPS = ["viridis", "plasma", "inferno", "magma", "thermal", "flame", "yellowy", "bipolar", "spectrum", "cyclic", "greyclip", "grey"]

# [修改] 更新 Range 選項: 移除 2.5，加入 4.0, 3.0, 2.0
RANGE_OPTIONS_AIR = [10.0, 5.0, 4.0, 3.0, 2.0, 1.0, 0.5, 0.1]
RANGE_OPTIONS_WATER = [40.0, 20.0, 10.0, 5.0, 4.0, 3.0, 2.0, 1.0]

# --- 輔助函式 ---
def read_packet(ser):
    if ser.in_waiting == 0: 
        return None
    
    header_bytes = ser.read(9)
    if len(header_bytes) != 9: return None
    if header_bytes[0] != 0xAA: return None 
    
    start, depth, freq_scaled, vDrv_scaled, num_samples = struct.unpack("<BHhHH", header_bytes)
    
    payload = ser.read(num_samples)
    if len(payload) != num_samples: return None
    
    checksum_bytes = ser.read(1)
    if len(checksum_bytes) != 1: return None
    
    calc_checksum = 0
    for b in header_bytes[1:]: calc_checksum ^= b
    for b in payload: calc_checksum ^= b
    
    if calc_checksum != checksum_bytes[0]: 
        return None
        
    values = np.frombuffer(payload, dtype=np.uint8, count=num_samples)
    return values, min(depth, num_samples), freq_scaled, float(vDrv_scaled)

def get_serial_ports():
    ports = [port.device for port in serial.tools.list_ports.comports()]
    return ports if ports else ["No Ports"]

def sonar_display_pipeline(raw_line, tvg_curve):
    line = raw_line.astype(np.float32)
    line = np.clip(line * DISPLAY_GAIN, 0, 255)
    out = line.copy()
    half = DESPECKLE_WINDOW // 2
    for i in range(half, len(line) - half):
        local = line[i - half:i + half + 1]
        local_mean = np.mean(local)
        if line[i] > local_mean + DESPECKLE_THRESHOLD and local_mean < DESPECKLE_THRESHOLD:
            out[i] = 0
    line = out
    for i in range(1, len(line)):
        line[i] = SMOOTH_ALPHA * line[i] + (1 - SMOOTH_ALPHA) * line[i - 1]
    
    if len(tvg_curve) == len(line):
        line *= tvg_curve
        
    return np.clip(line, 0, 255).astype(np.uint8)

# --- UI Classes ---
class BasePopup(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowFlags(Qt.Popup | Qt.FramelessWindowHint)
        self.setAttribute(Qt.WA_DeleteOnClose)
        self.font_size = SIDEBAR_BTN_FONT_SIZE - 3
        self.setStyleSheet(f"""
            QWidget {{ background-color: #2b2b2b; border: 1px solid #3e4145; }}
            QPushButton {{ background-color: transparent; color: #e0e0e0; font-family: 'Malgun Gothic'; font-size: {self.font_size}px; font-weight: bold; text-align: center; padding: 8px 5px; border: none; border-bottom: 1px solid #333; }}
            QPushButton:hover {{ background-color: #3e4145; color: white; }}
            QPushButton[active="true"] {{ background-color: #0078d7; color: white; }}
        """)

class ColorPopup(BasePopup):
    itemSelected = pyqtSignal(str)
    def __init__(self, current, parent=None):
        super().__init__(parent)
        layout = QVBoxLayout(self); layout.setContentsMargins(0, 0, 0, 0); layout.setSpacing(0)
        for name in COLOR_MAPS:
            btn = QPushButton(name.capitalize())
            if name == current: btn.setProperty("active", True)
            btn.clicked.connect(lambda checked, n=name: self.handle_click(n))
            layout.addWidget(btn)
        self.setFixedWidth(110)
    def handle_click(self, name): self.itemSelected.emit(name); self.close()

class RangePopup(BasePopup):
    itemSelected = pyqtSignal(float)
    def __init__(self, current_val, options, parent=None):
        super().__init__(parent)
        layout = QVBoxLayout(self); layout.setContentsMargins(0, 0, 0, 0); layout.setSpacing(0)
        total_depth_m = (parent.current_zoom_samples * SAMPLE_RESOLUTION) / 100.0
        current_step_approx = total_depth_m / 4.0
        for val in options:
            if val < 1.0: text = f"{val*100:.0f} cm"
            else: text = f"{val:.1f} m" if val % 1 != 0 else f"{val:.0f} m"
            btn = QPushButton(text)
            if abs(val - current_step_approx) < (val * 0.1): btn.setProperty("active", True)
            btn.clicked.connect(lambda checked, v=val: self.handle_click(v))
            layout.addWidget(btn)
        self.setFixedWidth(110)
    def handle_click(self, val): self.itemSelected.emit(val); self.close()

class CircularGauge(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedSize(40, 40); self.value = 1; self.max_value = 4
        self.bg_color = QColor("#333333"); self.progress_color = QColor("#00ff00") 
        self.setAttribute(Qt.WA_TranslucentBackground)
    def set_value(self, val): self.value = val; self.update()
    def paintEvent(self, event):
        painter = QPainter(self); painter.setRenderHint(QPainter.Antialiasing)
        w, h = self.width(), self.height(); padding = 4; rect = QRectF(padding, padding, w - 2*padding, h - 2*padding)
        painter.setPen(QPen(self.bg_color, 3, Qt.SolidLine, Qt.RoundCap)); painter.drawArc(rect, 225 * 16, -270 * 16)
        if self.max_value > 0:
            ratio = self.value / self.max_value; span = -270 * ratio * 16
            painter.setPen(QPen(self.progress_color, 3, Qt.SolidLine, Qt.RoundCap)); painter.drawArc(rect, 225 * 16, int(span))
        painter.setPen(Qt.white); font = QFont("Malgun Gothic", 13, QFont.Bold); painter.setFont(font)
        painter.drawText(rect, Qt.AlignCenter, str(self.value))

class GainGaugeWidget(QFrame):
    clicked = pyqtSignal()
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setObjectName("gainGaugeWidget"); self.setFixedHeight(60); self.setCursor(Qt.PointingHandCursor)
        layout = QHBoxLayout(self); layout.setContentsMargins(5, 5, 5, 5); layout.setSpacing(5)
        self.gauge = CircularGauge(); label_layout = QVBoxLayout(); label_layout.setSpacing(0); label_layout.addStretch()
        self.lbl_main = QLabel("LNA"); self.lbl_main.setObjectName("gainLabel")
        self.lbl_sub = QLabel("Gain"); self.lbl_sub.setObjectName("gainSubLabel")
        label_layout.addWidget(self.lbl_main); label_layout.addWidget(self.lbl_sub); label_layout.addStretch()
        layout.addWidget(self.gauge); layout.addLayout(label_layout)
    def set_value(self, val): self.gauge.set_value(val)
    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton: self.clicked.emit()

class SerialReader(QThread):
    data_received = pyqtSignal(np.ndarray, float, float, float)
    def __init__(self, port, baud_rate):
        super().__init__()
        self.port, self.baud_rate = port, baud_rate
        self.running = True
        self.send_queue = queue.Queue()
    def send_raw_command(self, cmd_bytes): self.send_queue.put(cmd_bytes)
    def stop(self): self.running = False; self.quit(); self.wait()
    def run(self):
        try:
            with serial.Serial(self.port, self.baud_rate, timeout=0.1, write_timeout=1) as ser:
                while self.running:
                    try:
                        while not self.send_queue.empty(): cmd = self.send_queue.get_nowait(); ser.write(cmd)
                    except: pass
                    result = read_packet(ser)
                    if result: self.data_received.emit(*result)
        except Exception as e: print(f"Serial Error: {e}")

class UDPReader(QThread):
    data_received = pyqtSignal(np.ndarray, float, float, float)
    def __init__(self, port): super().__init__(); self.port, self.running = port, True
    def stop(self): self.running = False; self.wait()
    def run(self):
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM); sock.settimeout(1.0); sock.bind(("", self.port))
            while self.running:
                try:
                    data, _ = sock.recvfrom(65536) 
                    pass 
                except: continue
        finally: sock.close()

# --- 設定對話框 ---
class SettingsDialog(QDialog):
    def __init__(self, parent):
        super().__init__(parent)
        self.main_app = parent
        self.setWindowTitle("Config"); self.resize(320, 580)
        self.echo_thr_val = self.main_app.saved_echo_thr
        
        self.setStyleSheet("""
            QDialog { background-color: #2b2b2b; color: #e0e0e0; font-family: 'Malgun Gothic', Arial; }
            QLabel { color: #e0e0e0; font-weight: bold; font-size: 11px; }
            QComboBox, QLineEdit { background-color: #3a3a3a; border: 1px solid #555; color: white; padding: 3px; border-radius: 2px; font-size: 11px; min-height: 18px; }
            QComboBox::drop-down { subcontrol-origin: padding; subcontrol-position: top right; width: 20px; border-left: 1px solid #555; background-color: #444; }
            QComboBox::down-arrow { width: 0px; height: 0px; border-left: 4px solid transparent; border-right: 4px solid transparent; border-top: 5px solid #ffffff; margin-top: 1px; margin-right: 1px; }
            QPushButton { background-color: #444; border: 1px solid #666; color: white; padding: 6px; border-radius: 3px; font-weight: bold; font-size: 12px; }
            QPushButton:hover { background-color: #555; border-color: #777; }
            QPushButton#applyBtn { background-color: #0078d7; border-color: #005a9e; }
            QPushButton#applyBtn:hover { background-color: #006cbd; }
            QGroupBox { border: 1px solid #555; border-radius: 4px; margin-top: 10px; padding-top: 5px; font-weight: bold; font-size: 12px; }
            QGroupBox::title { subcontrol-origin: margin; subcontrol-position: top left; left: 7px; padding: 0 2px; background-color: #2b2b2b; color: #00b4ff; }
            QScrollArea { border: none; background-color: transparent; }
            QWidget#scrollContent { background-color: transparent; }
        """)

        main_layout = QVBoxLayout(self); main_layout.setContentsMargins(2, 2, 2, 2); main_layout.setSpacing(2)
        scroll = QScrollArea(); scroll.setWidgetResizable(True); scroll_content = QWidget(); scroll_content.setObjectName("scrollContent")
        scroll_layout = QVBoxLayout(scroll_content); scroll_layout.setSpacing(8); scroll_layout.setContentsMargins(5, 5, 5, 5)

        # 1. Connection
        conn_group = QGroupBox("CONNECTION"); conn_layout = QFormLayout(conn_group); conn_layout.setContentsMargins(8, 8, 8, 8); conn_layout.setVerticalSpacing(6)
        self.source_combo = QComboBox(); self.source_combo.addItems(["Serial Port", "UDP Stream"]); self.source_combo.setCurrentText(self.main_app.connection_source)
        self.serial_combo = QComboBox(); self.serial_combo.addItems(get_serial_ports()); self.serial_combo.setCurrentText(self.main_app.serial_port_name)
        self.udp_port_input = QLineEdit(str(self.main_app.udp_port_num))
        
        self.res_combo = QComboBox()
        self.res_combo.addItems(["2000", "4000", "8000", "12000", "18000"])
        current_res_str = str(self.main_app.current_max_samples)
        index = self.res_combo.findText(current_res_str)
        if index >= 0: self.res_combo.setCurrentIndex(index)
        
        conn_layout.addRow("Type:", self.source_combo); 
        conn_layout.addRow("Port:", self.serial_combo); 
        conn_layout.addRow("UDP:", self.udp_port_input)
        conn_layout.addRow("Samples:", self.res_combo) 
        
        self.source_combo.currentTextChanged.connect(self.update_inputs); self.update_inputs(self.source_combo.currentText())
        scroll_layout.addWidget(conn_group)

        # 2. Display
        disp_group = QGroupBox("DISPLAY"); disp_layout = QFormLayout(disp_group); disp_layout.setContentsMargins(8, 8, 8, 8); disp_layout.setVerticalSpacing(6)
        self.speed_dropdown = QComboBox(); self.speed_dropdown.addItems([f"{AIR_SPEED} m/s (Air)", f"{WATER_SPEED} m/s (Water)"]); self.speed_dropdown.setCurrentIndex(1 if self.main_app.current_speed == WATER_SPEED else 0)
        self.large_depth_checkbox = QCheckBox("Show Depth"); self.large_depth_checkbox.setChecked(self.main_app.large_depth_visible)
        self.overlay_mode_combo = QComboBox(); self.overlay_mode_combo.addItems(["Auto (Threshold)", "Override (Max)"]); self.overlay_mode_combo.setCurrentIndex(0 if self.main_app.depth_overlay_mode == "Auto" else 1)
        self.show_line_checkbox = QCheckBox("Show Depth Profile"); self.show_line_checkbox.setChecked(self.main_app.show_depth_line)
        self.line_mode_combo = QComboBox(); self.line_mode_combo.addItems(["Follow Auto", "Follow Override"]); self.line_mode_combo.setCurrentIndex(0 if self.main_app.depth_line_mode == "Auto" else 1)
        disp_layout.addRow("Env:", self.speed_dropdown); disp_layout.addRow(self.large_depth_checkbox); disp_layout.addRow("Src:", self.overlay_mode_combo); disp_layout.addRow(self.show_line_checkbox); disp_layout.addRow("Line:", self.line_mode_combo)
        scroll_layout.addWidget(disp_group)

        # 3. NMEA
        nmea_group = QGroupBox("NMEA TCP"); nmea_layout = QFormLayout(nmea_group); nmea_layout.setContentsMargins(8, 8, 8, 8); nmea_layout.setVerticalSpacing(6)
        self.nmea_checkbox = QCheckBox("Enable"); self.nmea_checkbox.setChecked(self.main_app.nmea_output_enabled)
        self.nmea_port_input = QLineEdit(str(self.main_app.nmea_port))
        nmea_layout.addRow("On:", self.nmea_checkbox); nmea_layout.addRow("Port:", self.nmea_port_input)
        scroll_layout.addWidget(nmea_group)

        # 4. Register
        reg_group = QGroupBox("REGISTER"); reg_layout = QVBoxLayout(reg_group); reg_layout.setContentsMargins(8, 15, 8, 8); reg_layout.setSpacing(10)
        echo_layout = QHBoxLayout()
        self.lbl_echo_title = QLabel("Echo Thr:")
        self.btn_echo_minus = QPushButton("-"); self.btn_echo_minus.setFixedSize(25, 20); self.btn_echo_minus.clicked.connect(self.decrease_echo_thr)
        self.lbl_echo_val = QLabel(str(self.echo_thr_val)); self.lbl_echo_val.setAlignment(Qt.AlignCenter); self.lbl_echo_val.setFixedWidth(25)
        self.btn_echo_plus = QPushButton("+"); self.btn_echo_plus.setFixedSize(25, 20); self.btn_echo_plus.clicked.connect(self.increase_echo_thr)
        self.btn_echo_send = QPushButton("Send"); self.btn_echo_send.setCursor(Qt.PointingHandCursor); self.btn_echo_send.setFixedWidth(50); self.btn_echo_send.clicked.connect(self.send_echo_thr_cmd)
        echo_layout.addWidget(self.lbl_echo_title); echo_layout.addWidget(self.btn_echo_minus); echo_layout.addWidget(self.lbl_echo_val); echo_layout.addWidget(self.btn_echo_plus); echo_layout.addStretch(); echo_layout.addWidget(self.btn_echo_send)
        
        custom_layout = QHBoxLayout()
        self.lbl_raw_title = QLabel("Raw Reg:") 
        self.reg_input = QLineEdit(); self.reg_input.setPlaceholderText("Addr, Data")
        self.reg_btn = QPushButton("Send"); self.reg_btn.setCursor(Qt.PointingHandCursor); self.reg_btn.setFixedWidth(50); self.reg_btn.clicked.connect(self.handle_reg_send)
        custom_layout.addWidget(self.lbl_raw_title); custom_layout.addWidget(self.reg_input); custom_layout.addWidget(self.reg_btn) 

        reg_layout.addLayout(echo_layout); reg_layout.addLayout(custom_layout)
        scroll_layout.addWidget(reg_group); scroll_layout.addStretch(); scroll.setWidget(scroll_content); main_layout.addWidget(scroll)

        btn_layout = QHBoxLayout(); btn_layout.setContentsMargins(5, 0, 5, 5)
        apply_btn = QPushButton("Apply"); apply_btn.setObjectName("applyBtn"); apply_btn.clicked.connect(self.handle_apply)
        cancel_btn = QPushButton("Cancel"); cancel_btn.clicked.connect(self.close)
        btn_layout.addStretch(); btn_layout.addWidget(apply_btn); btn_layout.addWidget(cancel_btn); main_layout.addLayout(btn_layout)

    def decrease_echo_thr(self):
        if self.echo_thr_val > 1: self.echo_thr_val -= 1; self.lbl_echo_val.setText(str(self.echo_thr_val))
    def increase_echo_thr(self):
        if self.echo_thr_val < 16: self.echo_thr_val += 1; self.lbl_echo_val.setText(str(self.echo_thr_val))
    def send_echo_thr_cmd(self):
        if not self.main_app.serial_thread or not self.main_app.serial_thread.isRunning(): return print("[System] Not connected.")
        data = (self.echo_thr_val - 1) | 0x10; addr = 0x17
        try:
            self.main_app.serial_thread.send_raw_command(struct.pack('BBB', ord('W'), addr, data))
            self.main_app.saved_echo_thr = self.echo_thr_val
            print(f"[System] Echo Thr Sent: {self.echo_thr_val}")
        except Exception as e: print(f"[System] Error: {e}")

    def update_inputs(self, text):
        self.serial_combo.setEnabled(text == "Serial Port")
        self.udp_port_input.setEnabled(text != "Serial Port")

    def handle_reg_send(self):
        txt = self.reg_input.text().strip()
        if not self.main_app.serial_thread or not self.main_app.serial_thread.isRunning(): return print("[System] Not connected.")
        if ',' not in txt: return print("[System] Format Error")
        try:
            addr, data = [int(x.strip(), 16) for x in txt.split(',')]
            self.main_app.serial_thread.send_raw_command(struct.pack('BBB', ord('W'), addr, data))
            print(f"[System] Sent: Addr={hex(addr)}, Data={hex(data)}")
        except Exception as e: print(f"[System] Error: {e}")

    def handle_apply(self):
        self.main_app.connection_source = self.source_combo.currentText()
        self.main_app.serial_port_name = self.serial_combo.currentText()
        self.main_app.udp_port_num = int(self.udp_port_input.text()) if self.udp_port_input.text().isdigit() else 5005
        
        new_samples = int(self.res_combo.currentText())
        if new_samples != self.main_app.current_max_samples:
            self.main_app.change_resolution(new_samples)
            
        speed = AIR_SPEED if self.speed_dropdown.currentIndex() == 0 else WATER_SPEED
        self.main_app.set_sound_speed(speed)
        self.main_app.large_depth_visible = self.large_depth_checkbox.isChecked()
        self.main_app.depth_overlay.setVisible(self.main_app.large_depth_visible)
        self.main_app.depth_overlay_mode = "Auto" if self.overlay_mode_combo.currentIndex() == 0 else "Override"
        self.main_app.show_depth_line = self.show_line_checkbox.isChecked()
        self.main_app.depth_line_mode = "Auto" if self.line_mode_combo.currentIndex() == 0 else "Override"
        if not self.main_app.show_depth_line: self.main_app.depth_line.hide()
        port = int(self.nmea_port_input.text()) if self.nmea_port_input.text().isdigit() else 10110
        self.main_app.configure_nmea_output(self.nmea_checkbox.isChecked(), port)
        self.close()

# --- 主應用程式 ---
class WaterfallApp(QMainWindow):
    def __init__(self):
        super().__init__()
        self.serial_thread = None; self.udp_thread = None
        self.connection_source = "Serial Port"; self.serial_port_name = ""; ports = get_serial_ports()
        if ports: self.serial_port_name = ports[0]
        self.udp_port_num = 5005; self.is_connected = False
        self.nmea_output_enabled = False; self.nmea_port = 10110
        self.large_depth_visible = True; self.depth_overlay_mode = "Auto"
        self.show_depth_line = True; self.depth_line_mode = "Auto"
        self.current_gradient = 'cyclic'; 
        
        self.current_speed = SPEED_OF_SOUND 
        self.current_max_samples = 2000 
        self.current_zoom_samples = self.current_max_samples
        self.lna_gain = 4 
        self.saved_echo_thr = 16 
        self.min_zoom_samples = 20 
        
        self.tvg_curve = np.linspace(1.0, TVG_STRENGTH, self.current_max_samples)

        self.setWindowTitle("Open Echo Interface"); self.resize(900, 550)
        self.setStyleSheet(f"""
            * {{ font-family: 'Malgun Gothic', Arial, sans-serif; }}
            QMainWindow {{ background-color: black; }} QWidget {{ background-color: black; color: #e0e0e0; }}
            QFrame#sidebarFrame {{ background-color: #2b2b2b; border-left: 1px solid #1a1a1a; }}
            QPushButton.sidebar_btn {{ background-color: transparent; border: none; border-bottom: 1px solid #3e4145; color: #ccc; font-size: {SIDEBAR_BTN_FONT_SIZE}px; font-weight: bold; border-radius: 0px; padding: 10px; }}
            QPushButton.sidebar_btn:hover {{ background-color: #3e4145; color: white; }} QPushButton.sidebar_btn:pressed {{ background-color: #1a1a1a; color: #00aaff; }}
            QPushButton.connect_active {{ background-color: transparent; border: none; border-bottom: 1px solid #3e4145; border-left: 4px solid #ff5555; color: #ff5555; font-size: {SIDEBAR_BTN_FONT_SIZE}px; font-weight: bold; padding: 10px; }}
            QPushButton.connect_active:hover {{ background-color: #3e4145; }}
            QFrame#gainGaugeWidget {{ background-color: transparent; border-bottom: 1px solid #3e4145; }} QFrame#gainGaugeWidget:hover {{ background-color: #3e4145; }}
            QLabel#gainLabel {{ background-color: transparent; color: #ccc; font-weight: bold; font-size: {SIDEBAR_BTN_FONT_SIZE}px; }}
            QLabel#gainSubLabel {{ background-color: transparent; color: #ccc; font-size: {SIDEBAR_BTN_FONT_SIZE - 2}px; margin-top: -2px; }}
            QFrame#footerFrame {{ background-color: #111; border-top: 1px solid #333; }}
            QLabel#footerLabel {{ background-color: transparent; color: #aaa; font-size: 10px; margin-right: 15px; }}
        """)

        self.data = np.zeros((MAX_ROWS, self.current_max_samples))
        self.depth_history = np.full(MAX_ROWS, np.nan)
        
        central = QWidget(); self.setCentralWidget(central)
        main_layout = QHBoxLayout(central); main_layout.setContentsMargins(0, 0, 0, 0); main_layout.setSpacing(0)

        left_container = QWidget(); left_layout = QVBoxLayout(left_container); left_layout.setContentsMargins(0, 0, 0, 0); left_layout.setSpacing(0)
        self.waterfall = pg.PlotWidget(background='k'); vb = self.waterfall.getViewBox(); vb.setDefaultPadding(0)
        self.waterfall.setMouseEnabled(x=False, y=False); self.waterfall.showAxis('right'); self.waterfall.hideAxis('left'); self.waterfall.hideAxis('bottom')
        self.waterfall.setMenuEnabled(False); self.waterfall.getPlotItem().hideButtons()
        y_right = self.waterfall.getAxis('right'); y_right.setWidth(45); y_right.setStyle(showValues=True)
        y_right.setTextPen(pg.mkPen(color='w')); y_right.setPen(pg.mkPen(color=(100,100,100)))
        
        self.imageitem = pg.ImageItem(axisOrder="row-major"); self.waterfall.addItem(self.imageitem); self.waterfall.invertY(True)
        self.depth_overlay = pg.TextItem(anchor=(0, 1)); font = QFont("Malgun Gothic", 12); font.setWeight(QFont.DemiBold); self.depth_overlay.setFont(font); self.depth_overlay.setZValue(200); self.waterfall.addItem(self.depth_overlay)
        self.depth_line = pg.PlotCurveItem(pen=pg.mkPen(color='k', width=6)); self.depth_line.setZValue(50); self.waterfall.addItem(self.depth_line)
        
        self.set_sound_speed(self.current_speed) 
        
        left_layout.addWidget(self.waterfall)

        footer_frame = QFrame(); footer_frame.setObjectName("footerFrame"); footer_layout = QHBoxLayout(footer_frame); footer_layout.setContentsMargins(5, 2, 5, 2)
        self.lbl_footer_depth = QLabel("Depth: ---"); self.lbl_footer_depth.setObjectName("footerLabel")
        self.lbl_footer_ovr = QLabel("Override: ---"); self.lbl_footer_ovr.setObjectName("footerLabel")
        footer_layout.addWidget(self.lbl_footer_depth); footer_layout.addWidget(self.lbl_footer_ovr); footer_layout.addStretch()
        left_layout.addWidget(footer_frame); main_layout.addWidget(left_container, stretch=1)

        self.colorbar = pg.HistogramLUTWidget(); self.colorbar.setImageItem(self.imageitem); 
        self.colorbar.item.gradient.loadPreset(self.current_gradient)
        sidebar = QFrame(); sidebar.setObjectName("sidebarFrame"); sidebar.setFixedWidth(110)
        side_layout = QVBoxLayout(sidebar); side_layout.setContentsMargins(0, 0, 0, 0); side_layout.setSpacing(0)

        self.btn_plus = QPushButton("+"); self.btn_plus.setProperty("class", "sidebar_btn"); self.btn_plus.setFixedHeight(60); self.btn_plus.clicked.connect(self.zoom_in); side_layout.addWidget(self.btn_plus)
        self.btn_minus = QPushButton("-"); self.btn_minus.setProperty("class", "sidebar_btn"); self.btn_minus.setFixedHeight(60); self.btn_minus.clicked.connect(self.zoom_out); side_layout.addWidget(self.btn_minus)
        
        self.btn_range = QPushButton("Range"); self.btn_range.setProperty("class", "sidebar_btn"); self.btn_range.setFixedHeight(60); self.btn_range.clicked.connect(self.show_range_menu); side_layout.addWidget(self.btn_range)
        self.btn_color = QPushButton("Color"); self.btn_color.setProperty("class", "sidebar_btn"); self.btn_color.setFixedHeight(60); self.btn_color.clicked.connect(self.show_color_menu); side_layout.addWidget(self.btn_color)
        
        self.lna_widget = GainGaugeWidget(); self.lna_widget.set_value(self.lna_gain); self.lna_widget.clicked.connect(self.cycle_lna_gain); side_layout.addWidget(self.lna_widget)
        spacer = QWidget(); spacer.setSizePolicy(QSizePolicy.Minimum, QSizePolicy.Expanding); spacer.setStyleSheet("background-color: transparent;"); side_layout.addWidget(spacer)
        
        self.btn_settings = QPushButton("Settings"); self.btn_settings.setProperty("class", "sidebar_btn"); self.btn_settings.setFixedHeight(60); self.btn_settings.clicked.connect(self.open_settings); side_layout.addWidget(self.btn_settings)
        self.btn_connect = QPushButton("Connect"); self.btn_connect.setProperty("class", "sidebar_btn"); self.btn_connect.setFixedHeight(60); self.btn_connect.clicked.connect(self.handle_main_connect); side_layout.addWidget(self.btn_connect)
        main_layout.addWidget(sidebar)

    def change_resolution(self, new_samples):
        print(f"[System] Changing resolution to {new_samples} samples")
        self.current_max_samples = new_samples
        
        self.data = np.zeros((MAX_ROWS, self.current_max_samples))
        
        # [修改] 重置深度線
        self.depth_history = np.full(MAX_ROWS, np.nan)
        self.depth_line.setData(x=np.arange(MAX_ROWS), y=self.depth_history, connect="finite")
        
        self.tvg_curve = np.linspace(1.0, TVG_STRENGTH, self.current_max_samples)
        
        if self.serial_thread and self.serial_thread.isRunning():
            cmd = struct.pack('>B H', ord('N'), new_samples)
            self.serial_thread.send_raw_command(cmd)
            
        self.current_zoom_samples = self.current_max_samples
        self.update_zoom_range()

    def show_color_menu(self):
        btn_pos = self.btn_color.mapToGlobal(QPoint(0, 0)); popup = ColorPopup(self.current_gradient, self); popup.itemSelected.connect(self.set_gradient)
        self.position_popup(popup, btn_pos)

    def show_range_menu(self):
        btn_pos = self.btn_range.mapToGlobal(QPoint(0, 0))
        raw_options = RANGE_OPTIONS_AIR if self.current_speed == AIR_SPEED else RANGE_OPTIONS_WATER
        max_phys_depth = (self.current_max_samples * SAMPLE_RESOLUTION) / 100.0
        
        valid_options = [opt for opt in raw_options if (opt * 4.0) <= max_phys_depth]
        if not valid_options: valid_options = [raw_options[-1]]
            
        popup = RangePopup(self.current_zoom_samples, valid_options, self)
        popup.itemSelected.connect(self.set_range_step)
        self.position_popup(popup, btn_pos)

    def position_popup(self, popup, btn_pos):
        popup.adjustSize(); x = btn_pos.x() - popup.width(); y = btn_pos.y()
        screen_geo = QApplication.desktop().availableGeometry(btn_pos)
        if y + popup.height() > screen_geo.bottom(): y = screen_geo.bottom() - popup.height() - 5
        if y < screen_geo.top(): y = screen_geo.top() + 5
        popup.move(x, y); popup.show()

    def set_gradient(self, n): self.current_gradient = n; self.colorbar.item.gradient.loadPreset(n)
    
    def set_range_step(self, val):
        total_depth = val * 4.0
        self.current_zoom_samples = (total_depth * 100.0) / SAMPLE_RESOLUTION
        self.update_zoom_range()

    def cycle_lna_gain(self):
        self.lna_gain += 1; 
        if self.lna_gain > 4: self.lna_gain = 1
        self.lna_widget.set_value(self.lna_gain)
        if self.serial_thread and self.serial_thread.isRunning():
            reg_map = {1: 0x05, 2: 0x07, 3: 0x04, 4: 0x06}
            val = reg_map.get(self.lna_gain, 0x06)
            self.serial_thread.send_raw_command(struct.pack('BBB', ord('W'), 0x13, val))

    def zoom_in(self): 
        min_range_step = 0.1 
        min_total_depth = min_range_step * 4.0
        min_allowed_samples = (min_total_depth * 100.0) / SAMPLE_RESOLUTION
        
        if self.current_zoom_samples <= min_allowed_samples + 1.0: 
            return

        step = 200
        self.current_zoom_samples = max(min_allowed_samples, self.current_zoom_samples - step)
        self.update_zoom_range()
    
    def zoom_out(self): 
        step = 200
        self.current_zoom_samples = min(self.current_max_samples, self.current_zoom_samples + step)
        self.update_zoom_range()

    def update_zoom_range(self):
        pad_top = self.current_zoom_samples * 0.02; pad_bottom = self.current_zoom_samples * 0.02 
        self.waterfall.setYRange(-pad_top, self.current_zoom_samples + pad_bottom, padding=0)
        overlay_pos = self.current_zoom_samples - (self.current_zoom_samples * 0.05); self.depth_overlay.setPos(10, overlay_pos)
        
        total_depth_m = (self.current_zoom_samples * SAMPLE_RESOLUTION) / 100.0
        step_m = total_depth_m / 4.0
        
        tick_depths = [i * step_m for i in range(5)]
        
        ticks = []
        fmt = "{:.1f}" if step_m < 1 else "{:.1f}" 
        
        for d in tick_depths:
            idx = (d * 100.0) / SAMPLE_RESOLUTION
            ticks.append((idx, fmt.format(d)))
            
        ax = self.waterfall.getAxis("right"); ax.setTicks([ticks])
        
        for item in self.waterfall.items():
            if isinstance(item, pg.InfiniteLine) and item != self.depth_line: self.waterfall.removeItem(item)
        for idx, label in ticks:
            if idx > 0: 
                line = pg.InfiniteLine(pos=idx, angle=0, pen=pg.mkPen(color=(150,150,150,150), style=Qt.DashLine))
                self.waterfall.addItem(line)

    def waterfall_plot_callback(self, raw_data, depth_index, drive_frequency, override_idx):
        if len(raw_data) != self.current_max_samples:
            if abs(len(raw_data) - self.current_max_samples) > 0:
                 self.current_max_samples = len(raw_data)
                 self.data = np.zeros((MAX_ROWS, self.current_max_samples))
                 self.tvg_curve = np.linspace(1.0, TVG_STRENGTH, self.current_max_samples)
                 # [修改] 自動重置深度線 (當收到非預期長度的封包時)
                 self.depth_history = np.full(MAX_ROWS, np.nan)
                 self.depth_line.setData(x=np.arange(MAX_ROWS), y=self.depth_history, connect="finite")

        filtered = sonar_display_pipeline(raw_data, self.tvg_curve)
        self.data = np.roll(self.data, -1, axis=0); self.data[-1, :] = filtered
        self.imageitem.setImage(self.data.T, autoLevels=False); self.imageitem.setLevels((10, 220))
        depth_m, ovr_m = (depth_index * SAMPLE_RESOLUTION) / 100.0, (override_idx * SAMPLE_RESOLUTION) / 100.0
        
        not_blind = (depth_index > PYTHON_IGNORE_INDEX)
        diff = abs(int(depth_index) - int(override_idx))
        is_consistent = (diff <= INDEX_TOLERANCE)
        is_reliable = not_blind and is_consistent
        target_depth_idx = depth_index if self.depth_line_mode == "Auto" else override_idx
        val_to_plot = target_depth_idx if is_reliable else np.nan

        self.depth_history = np.roll(self.depth_history, -1); self.depth_history[-1] = val_to_plot
        if self.show_depth_line:
            self.depth_line.setData(x=np.arange(MAX_ROWS), y=self.depth_history, connect="finite"); self.depth_line.show()
        else: self.depth_line.hide()

        if self.large_depth_visible:
            color_hex = "#FFFFFF" if is_reliable else "rgba(255, 255, 255, 0.2)" 
            display_val_m = (target_depth_idx * SAMPLE_RESOLUTION) / 100.0
            freq_text = f"&nbsp;&nbsp;{drive_frequency:.0f}kHz"
            html_str = f"""<div style="text-align: left; line-height: 90%; font-family: 'Malgun Gothic';"><span style="font-size: 64pt; font-weight: 600; color: {color_hex};">{display_val_m:.1f}</span><span style="font-size: 32pt; font-weight: 600; color: {color_hex};">m</span><br><span style="font-size: 10pt; color: #cccccc; font-weight: 600;">{freq_text}</span></div>"""
            self.depth_overlay.setHtml(html_str)

        self.lbl_footer_depth.setText(f"Depth: {depth_m * 100:.0f} cm"); self.lbl_footer_ovr.setText(f"Override: {ovr_m * 100:.0f} cm")

    def set_sound_speed(self, speed):
        global SPEED_OF_SOUND, SAMPLE_RESOLUTION, MAX_DEPTH
        SPEED_OF_SOUND = self.current_speed = speed
        SAMPLE_RESOLUTION = (SPEED_OF_SOUND * SAMPLE_TIME * 100) / 2
        MAX_DEPTH = self.current_max_samples * SAMPLE_RESOLUTION 
        ax = self.waterfall.getAxis("right"); ax.setTickFont(QFont("Arial", 10))
        
        raw_options = RANGE_OPTIONS_AIR if speed == AIR_SPEED else RANGE_OPTIONS_WATER
        max_phys_depth = (self.current_max_samples * SAMPLE_RESOLUTION) / 100.0
        
        valid_options = [opt for opt in raw_options if (opt * 4.0) <= max_phys_depth]
        if valid_options:
            init_interval = valid_options[0] 
        else:
            init_interval = raw_options[-1] 
            
        self.current_zoom_samples = (init_interval * 4.0 * 100.0) / SAMPLE_RESOLUTION
        self.update_zoom_range()

    def handle_main_connect(self):
        if self.is_connected:
            if self.serial_thread: self.serial_thread.stop(); self.serial_thread = None
            if self.udp_thread: self.udp_thread.stop(); self.udp_thread = None
            self.is_connected = False; self.btn_connect.setText("Connect"); self.btn_connect.setProperty("class", "sidebar_btn"); self.btn_connect.setStyle(self.btn_connect.style()) 
            self.lbl_footer_depth.setText("Depth: ---"); self.lbl_footer_ovr.setText("Override: ---")
        else:
            if self.connection_source == "Serial Port":
                if not self.serial_port_name or self.serial_port_name == "No Ports": return print("No Serial Port Selected")
                self.serial_thread = SerialReader(self.serial_port_name, BAUD_RATE); self.serial_thread.data_received.connect(self.waterfall_plot_callback); self.serial_thread.start()
                
                QThread.msleep(2000) 
                
                cmd = struct.pack('>B H', ord('N'), self.current_max_samples)
                self.serial_thread.send_raw_command(cmd)
                QThread.msleep(100) 
                
                reg_map = {1: 0x05, 2: 0x07, 3: 0x04, 4: 0x06}
                val = reg_map.get(self.lna_gain, 0x06)
                self.serial_thread.send_raw_command(struct.pack('BBB', ord('W'), 0x13, val))
                
                data = (self.saved_echo_thr - 1) | 0x10; addr = 0x17
                self.serial_thread.send_raw_command(struct.pack('BBB', ord('W'), addr, data))
                
            else:
                try: self.udp_thread = UDPReader(self.udp_port_num); self.udp_thread.data_received.connect(self.waterfall_plot_callback); self.udp_thread.start()
                except: return
            self.is_connected = True; self.btn_connect.setText("Stop"); self.btn_connect.setProperty("class", "connect_active"); self.btn_connect.setStyle(self.btn_connect.style())

    def open_settings(self): dlg = SettingsDialog(self); dlg.exec_()
    def set_gradient(self, n): self.current_gradient = n; self.colorbar.item.gradient.loadPreset(n)
    def configure_nmea_output(self, e, p): self.nmea_output_enabled, self.nmea_port = e, p
    def closeEvent(self, e):
        if self.serial_thread: self.serial_thread.stop()
        if self.udp_thread: self.udp_thread.stop()
        e.accept()

if __name__ == "__main__":
    os.environ["QT_AUTO_SCREEN_SCALE_FACTOR"] = "1"
    QApplication.setAttribute(Qt.AA_EnableHighDpiScaling)
    QApplication.setAttribute(Qt.AA_UseHighDpiPixmaps)
    app = QApplication(sys.argv)
    font = app.font(); font.setPointSize(10); app.setFont(font)
    window = WaterfallApp(); window.show(); sys.exit(app.exec())
