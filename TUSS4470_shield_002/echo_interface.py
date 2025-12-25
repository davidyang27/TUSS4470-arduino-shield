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
from PyQt5.QtCore import QThread, pyqtSignal, Qt, QSize
from PyQt5.QtGui import QPalette, QColor, QFont
import pyqtgraph as pg

# ============================================================
# --- 全域配置參數 ---
# ============================================================
AIR_SPEED = 343.0      
WATER_SPEED = 1440.0   
DEFAULT_ENVIRONMENT = 'AIR' 

BAUD_RATE = 250000
NUM_SAMPLES = 1800
INDEX_TOLERANCE = 10
MAX_ROWS = 300
Y_LABEL_DISTANCE = 50  
SAMPLE_TIME = 13.2e-6
DEFAULT_LEVELS = (0, 256)

SPEED_OF_SOUND = AIR_SPEED if DEFAULT_ENVIRONMENT == 'AIR' else WATER_SPEED 
SAMPLE_RESOLUTION = (SPEED_OF_SOUND * SAMPLE_TIME * 100) / 2
MAX_DEPTH = NUM_SAMPLES * SAMPLE_RESOLUTION

DISPLAY_GAIN = 1.3
DESPECKLE_WINDOW = 3
DESPECKLE_THRESHOLD = 10
SMOOTH_ALPHA = 0.25
TVG_STRENGTH = 1.2

# --- 輔助函式 ---
def read_packet(ser):
    while True:
        header = ser.read(1)
        if header != b"\xaa": continue
        payload = ser.read(6 + NUM_SAMPLES)
        checksum = ser.read(1)
        if len(payload) != 6 + NUM_SAMPLES or len(checksum) != 1: continue
        calc_checksum = 0
        for byte in payload: calc_checksum ^= byte
        if calc_checksum != checksum[0]: continue
        depth, freq_scaled, vDrv_scaled = struct.unpack("<HhH", payload[:6])
        sample_bytes = payload[6:6+NUM_SAMPLES]
        values = np.frombuffer(sample_bytes, dtype=np.uint8, count=NUM_SAMPLES)
        return values, min(depth, NUM_SAMPLES), freq_scaled, float(vDrv_scaled)

def get_serial_ports():
    ports = [port.device for port in serial.tools.list_ports.comports()]
    return ports if ports else ["No Ports"]

def get_local_ip():
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except: return "127.0.0.1"

def sonar_display_pipeline(raw_line):
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
    depth_gain = np.linspace(1.0, TVG_STRENGTH, len(line))
    line *= depth_gain
    return np.clip(line, 0, 255).astype(np.uint8)

# --- 執行緒類別 ---
class SerialReader(QThread):
    data_received = pyqtSignal(np.ndarray, float, float, float)
    def __init__(self, port, baud_rate):
        super().__init__()
        self.port, self.baud_rate = port, baud_rate
        self.running = True
        self.send_queue = queue.Queue()
    def send_raw_command(self, cmd_bytes): self.send_queue.put(cmd_bytes)
    def stop(self):
        self.running = False
        self.quit(); self.wait()
    def run(self):
        try:
            with serial.Serial(self.port, self.baud_rate, timeout=0.1, write_timeout=1) as ser:
                while self.running:
                    try:
                        while not self.send_queue.empty(): ser.write(self.send_queue.get_nowait())
                    except: pass
                    result = read_packet(ser)
                    if result: self.data_received.emit(*result)
        except Exception as e: print(f"Serial Error: {e}")

class UDPReader(QThread):
    data_received = pyqtSignal(np.ndarray, float, float, float)
    def __init__(self, port):
        super().__init__()
        self.port, self.running = port, True
    def stop(self): self.running = False; self.wait()
    def run(self):
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.settimeout(1.0); sock.bind(("", self.port))
            while self.running:
                try:
                    data, _ = sock.recvfrom(1 + 6 + NUM_SAMPLES + 100)
                    if data[0] == 0xAA:
                        payload = data[1:1+6+NUM_SAMPLES]
                        d, f, v = struct.unpack("<HhH", payload[:6])
                        self.data_received.emit(np.frombuffer(payload[6:], dtype=np.uint8), d, f, float(v))
                except: continue
        finally: sock.close()

# --- 設定對話框 ---
class SettingsDialog(QDialog):
    def __init__(self, parent):
        super().__init__(parent)
        self.main_app = parent
        self.setWindowTitle("Config")
        self.resize(320, 520) 
        
        self.setStyleSheet("""
            QDialog { background-color: #2b2b2b; color: #e0e0e0; font-family: 'Malgun Gothic', Arial; }
            QLabel { color: #e0e0e0; font-weight: bold; font-size: 11px; }
            QComboBox, QLineEdit { 
                background-color: #3a3a3a; border: 1px solid #555; color: white; 
                padding: 3px; border-radius: 2px; font-size: 11px; min-height: 18px;
            }
            QComboBox::drop-down { 
                subcontrol-origin: padding; subcontrol-position: top right; width: 20px;
                border-left: 1px solid #555; border-top-right-radius: 2px; border-bottom-right-radius: 2px;
                background-color: #444;
            }
            QComboBox::down-arrow {
                width: 0px; height: 0px;
                border-left: 4px solid transparent; border-right: 4px solid transparent;
                border-top: 5px solid #ffffff; margin-top: 1px; margin-right: 1px;
            }
            QPushButton { 
                background-color: #444; border: 1px solid #666; color: white; 
                padding: 6px; border-radius: 3px; font-weight: bold; font-size: 12px;
            }
            QPushButton:hover { background-color: #555; border-color: #777; }
            QPushButton#applyBtn { background-color: #0078d7; border-color: #005a9e; }
            QPushButton#applyBtn:hover { background-color: #006cbd; }
            QGroupBox { 
                border: 1px solid #555; border-radius: 4px; margin-top: 10px; padding-top: 5px; 
                font-weight: bold; font-size: 12px; 
            }
            QGroupBox::title { 
                subcontrol-origin: margin; subcontrol-position: top left; left: 7px; padding: 0 2px; 
                background-color: #2b2b2b; color: #00b4ff; 
            }
            QScrollArea { border: none; background-color: transparent; }
            QWidget#scrollContent { background-color: transparent; }
        """)

        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(2, 2, 2, 2)
        main_layout.setSpacing(2)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll_content = QWidget(); scroll_content.setObjectName("scrollContent")
        scroll_layout = QVBoxLayout(scroll_content)
        scroll_layout.setSpacing(8)
        scroll_layout.setContentsMargins(5, 5, 5, 5)

        # 1. Connection
        conn_group = QGroupBox("CONNECTION")
        conn_layout = QFormLayout(conn_group); conn_layout.setContentsMargins(8, 8, 8, 8); conn_layout.setVerticalSpacing(6)
        self.source_combo = QComboBox(); self.source_combo.addItems(["Serial Port", "UDP Stream"]); self.source_combo.setCurrentText(self.main_app.connection_source)
        self.serial_combo = QComboBox(); self.serial_combo.addItems(get_serial_ports()); self.serial_combo.setCurrentText(self.main_app.serial_port_name)
        self.udp_port_input = QLineEdit(str(self.main_app.udp_port_num))
        conn_layout.addRow("Type:", self.source_combo); conn_layout.addRow("Port:", self.serial_combo); conn_layout.addRow("UDP:", self.udp_port_input)
        self.source_combo.currentTextChanged.connect(self.update_inputs); self.update_inputs(self.source_combo.currentText())
        scroll_layout.addWidget(conn_group)

        # 2. Display
        app_group = QGroupBox("DISPLAY")
        app_layout = QFormLayout(app_group); app_layout.setContentsMargins(8, 8, 8, 8); app_layout.setVerticalSpacing(6)
        self.speed_dropdown = QComboBox(); self.speed_dropdown.addItems([f"{AIR_SPEED} m/s (Air)", f"{WATER_SPEED} m/s (Water)"]); self.speed_dropdown.setCurrentIndex(1 if self.main_app.current_speed == WATER_SPEED else 0)
        self.gradient_dropdown = QComboBox(); self.gradient_dropdown.addItems(["viridis", "plasma", "inferno", "magma", "thermal", "flame", "yellowy", "bipolar", "spectrum", "cyclic", "greyclip", "grey"]); self.gradient_dropdown.setCurrentText(self.main_app.current_gradient)
        app_layout.addRow("Env:", self.speed_dropdown); app_layout.addRow("Color:", self.gradient_dropdown)
        scroll_layout.addWidget(app_group)

        # 3. Overlay
        disp_group = QGroupBox("OVERLAY")
        disp_layout = QFormLayout(disp_group); disp_layout.setContentsMargins(8, 8, 8, 8); disp_layout.setVerticalSpacing(6)
        self.large_depth_checkbox = QCheckBox("Show Depth"); self.large_depth_checkbox.setChecked(self.main_app.large_depth_visible)
        self.overlay_mode_combo = QComboBox(); self.overlay_mode_combo.addItems(["Auto (Threshold)", "Override (Max)"]); self.overlay_mode_combo.setCurrentIndex(0 if self.main_app.depth_overlay_mode == "Auto" else 1)
        self.show_line_checkbox = QCheckBox("Show Red Line"); self.show_line_checkbox.setChecked(self.main_app.show_depth_line)
        self.line_mode_combo = QComboBox(); self.line_mode_combo.addItems(["Follow Auto", "Follow Override"]); self.line_mode_combo.setCurrentIndex(0 if self.main_app.depth_line_mode == "Auto" else 1)
        disp_layout.addRow(self.large_depth_checkbox); disp_layout.addRow("Src:", self.overlay_mode_combo); disp_layout.addRow(self.show_line_checkbox); disp_layout.addRow("Line:", self.line_mode_combo)
        scroll_layout.addWidget(disp_group)

        # 4. NMEA
        nmea_group = QGroupBox("NMEA TCP")
        nmea_layout = QFormLayout(nmea_group); nmea_layout.setContentsMargins(8, 8, 8, 8); nmea_layout.setVerticalSpacing(6)
        self.nmea_checkbox = QCheckBox("Enable"); self.nmea_checkbox.setChecked(self.main_app.nmea_output_enabled)
        self.nmea_port_input = QLineEdit(str(self.main_app.nmea_port))
        nmea_layout.addRow("On:", self.nmea_checkbox); nmea_layout.addRow("Port:", self.nmea_port_input)
        scroll_layout.addWidget(nmea_group)

        # 5. Register
        reg_group = QGroupBox("REGISTER (HEX)")
        reg_layout = QHBoxLayout(reg_group); reg_layout.setContentsMargins(8, 15, 8, 8)
        self.reg_input = QLineEdit(); self.reg_input.setPlaceholderText("Addr, Data")
        self.reg_btn = QPushButton("Send"); self.reg_btn.clicked.connect(self.handle_reg_send)
        reg_layout.addWidget(self.reg_input); reg_layout.addWidget(self.reg_btn)
        scroll_layout.addWidget(reg_group)

        scroll_layout.addStretch()
        scroll.setWidget(scroll_content)
        main_layout.addWidget(scroll)

        # Buttons
        btn_layout = QHBoxLayout(); btn_layout.setContentsMargins(5, 0, 5, 5)
        apply_btn = QPushButton("Apply"); apply_btn.setObjectName("applyBtn"); apply_btn.clicked.connect(self.handle_apply)
        cancel_btn = QPushButton("Cancel"); cancel_btn.clicked.connect(self.close)
        btn_layout.addStretch(); btn_layout.addWidget(apply_btn); btn_layout.addWidget(cancel_btn)
        main_layout.addLayout(btn_layout)

    def update_inputs(self, text):
        is_serial = (text == "Serial Port")
        self.serial_combo.setEnabled(is_serial)
        self.udp_port_input.setEnabled(not is_serial)

    def handle_reg_send(self):
        txt = self.reg_input.text().strip()
        if ',' in txt and self.main_app.serial_thread:
            try:
                addr, data = [int(x.strip(), 16) for x in txt.split(',')]
                if self.main_app.serial_thread and self.main_app.serial_thread.isRunning():
                    self.main_app.serial_thread.send_raw_command(struct.pack('BBB', ord('W'), addr, data))
            except: pass

    def handle_apply(self):
        self.main_app.connection_source = self.source_combo.currentText()
        self.main_app.serial_port_name = self.serial_combo.currentText()
        self.main_app.udp_port_num = int(self.udp_port_input.text()) if self.udp_port_input.text().isdigit() else 5005
        speed = AIR_SPEED if self.speed_dropdown.currentIndex() == 0 else WATER_SPEED
        self.main_app.set_sound_speed(speed)
        self.main_app.set_gradient(self.gradient_dropdown.currentText())
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
        self.current_gradient = 'cyclic'; self.current_speed = SPEED_OF_SOUND 
        
        self.current_zoom_samples = NUM_SAMPLES 
        self.min_zoom_samples = 200 
        
        self.setWindowTitle("Open Echo Interface")
        self.resize(900, 550)
        
        self.setStyleSheet("""
            * { font-family: 'Malgun Gothic', Arial, sans-serif; }
            QMainWindow { background-color: black; } 
            QWidget { background-color: black; color: #e0e0e0; }
            QFrame#sidebarFrame { background-color: #2b2b2b; border-left: 1px solid #1a1a1a; }
            
            QPushButton.sidebar_btn {
                background-color: transparent; 
                border: none;
                border-bottom: 1px solid #3e4145; 
                color: #ccc; font-size: 15px; font-weight: bold; border-radius: 0px; padding: 10px;
            }
            QPushButton.sidebar_btn:hover { background-color: #3e4145; color: white; }
            QPushButton.sidebar_btn:pressed { background-color: #1a1a1a; color: #00aaff; }

            QPushButton.connect_active {
                background-color: transparent; border: none; border-bottom: 1px solid #3e4145;
                border-left: 4px solid #ff5555; color: #ff5555; font-size: 15px; font-weight: bold; padding: 10px;
            }
            QPushButton.connect_active:hover { background-color: #3e4145; }
            
            QFrame#statusFrame { 
                background-color: transparent; border: none; border-bottom: 1px solid #3e4145; 
                margin: 0px; padding: 5px;
            }
            QLabel#statusTitle { color: #888; font-size: 12px; font-weight: bold; margin-bottom: 2px; }
            QLabel#statusValue { color: #00ff00; font-family: Consolas, Monospace; font-size: 13px; font-weight: bold; }
        """)

        self.data = np.zeros((MAX_ROWS, NUM_SAMPLES))
        
        self.depth_history = np.full(MAX_ROWS, np.nan)
        self.valid_history = np.zeros(MAX_ROWS, dtype=bool) 
        
        central = QWidget(); self.setCentralWidget(central)
        main_layout = QHBoxLayout(central); main_layout.setContentsMargins(0, 0, 0, 0); main_layout.setSpacing(0)

        # ------------------ 左側：聲納圖 ------------------
        self.waterfall = pg.PlotWidget(background='k')
        vb = self.waterfall.getViewBox()
        vb.setDefaultPadding(0)
        self.waterfall.setMouseEnabled(x=False, y=False)
        self.waterfall.showAxis('right'); self.waterfall.hideAxis('left'); self.waterfall.hideAxis('bottom')
        
        y_right = self.waterfall.getAxis('right')
        y_right.setWidth(60) 
        y_right.setStyle(showValues=True)
        y_right.setTextPen(pg.mkPen(color='w'))
        y_right.setPen(pg.mkPen(color=(100,100,100)))

        self.imageitem = pg.ImageItem(axisOrder="row-major"); self.waterfall.addItem(self.imageitem); self.waterfall.invertY(True)
        
        self.depth_overlay = pg.TextItem(anchor=(0, 1)) 
        font = QFont("Malgun Gothic", 12)
        font.setWeight(QFont.DemiBold) 
        self.depth_overlay.setFont(font)
        self.depth_overlay.setZValue(200)
        self.waterfall.addItem(self.depth_overlay)
        
        # [修改] 線條加粗為 6
        self.depth_line = pg.PlotCurveItem(pen=pg.mkPen(color='k', width=6))
        self.depth_line.setZValue(50)
        self.waterfall.addItem(self.depth_line)
        
        self.update_zoom_range()
        self.set_sound_speed(self.current_speed)

        main_layout.addWidget(self.waterfall, stretch=1)

        # ------------------ 右側：扁平化側邊欄 ------------------
        sidebar = QFrame(); sidebar.setObjectName("sidebarFrame")
        sidebar.setFixedWidth(110)
        side_layout = QVBoxLayout(sidebar); side_layout.setContentsMargins(0, 0, 0, 0); side_layout.setSpacing(0)

        # 1. Zoom
        self.btn_plus = QPushButton("+"); self.btn_plus.setProperty("class", "sidebar_btn"); self.btn_plus.setFixedHeight(60)
        self.btn_plus.clicked.connect(self.zoom_in)
        side_layout.addWidget(self.btn_plus)
        self.btn_minus = QPushButton("-"); self.btn_minus.setProperty("class", "sidebar_btn"); self.btn_minus.setFixedHeight(60)
        self.btn_minus.clicked.connect(self.zoom_out)
        side_layout.addWidget(self.btn_minus)
        
        # 2. Status
        status_box = QFrame(); status_box.setObjectName("statusFrame")
        status_layout = QVBoxLayout(status_box); status_layout.setContentsMargins(10, 10, 10, 10); status_layout.setSpacing(2)
        lbl_title = QLabel("STATUS"); lbl_title.setObjectName("statusTitle"); lbl_title.setAlignment(Qt.AlignCenter)
        status_layout.addWidget(lbl_title)
        
        self.lbl_depth = QLabel("Dep: ---"); self.lbl_depth.setObjectName("statusValue")
        self.lbl_ovr = QLabel("Ovr: ---"); self.lbl_ovr.setObjectName("statusValue")
        
        status_layout.addSpacing(5)
        status_layout.addWidget(self.lbl_depth); status_layout.addWidget(self.lbl_ovr)
        side_layout.addWidget(status_box)
        
        # 3. Spacer & Functions
        spacer = QWidget(); spacer.setSizePolicy(QSizePolicy.Minimum, QSizePolicy.Expanding); spacer.setStyleSheet("background-color: transparent;"); side_layout.addWidget(spacer)
        self.btn_settings = QPushButton("Settings"); self.btn_settings.setProperty("class", "sidebar_btn"); self.btn_settings.setFixedHeight(60); self.btn_settings.clicked.connect(self.open_settings); side_layout.addWidget(self.btn_settings)
        self.btn_connect = QPushButton("Connect"); self.btn_connect.setProperty("class", "sidebar_btn"); self.btn_connect.setFixedHeight(60); self.btn_connect.clicked.connect(self.handle_main_connect); side_layout.addWidget(self.btn_connect)

        main_layout.addWidget(sidebar)
        self.colorbar = pg.HistogramLUTWidget(); self.colorbar.setImageItem(self.imageitem); self.colorbar.item.gradient.loadPreset("cyclic"); self.imageitem.setLevels(DEFAULT_LEVELS)

    def zoom_in(self):
        self.current_zoom_samples = max(self.min_zoom_samples, self.current_zoom_samples - 200)
        self.update_zoom_range()

    def zoom_out(self):
        self.current_zoom_samples = min(NUM_SAMPLES, self.current_zoom_samples + 200)
        self.update_zoom_range()

    def update_zoom_range(self):
        pad_top = self.current_zoom_samples * 0.02; pad_bottom = self.current_zoom_samples * 0.02 
        self.waterfall.setYRange(-pad_top, self.current_zoom_samples + pad_bottom, padding=0)
        tick_indices = np.linspace(0, self.current_zoom_samples, 9)
        ticks = []
        for idx in tick_indices:
            depth_m = (idx * SAMPLE_RESOLUTION) / 100.0
            ticks.append((idx, f"{depth_m:.1f}"))
        ax = self.waterfall.getAxis("right"); ax.setTicks([ticks])
        for item in self.waterfall.items():
            if isinstance(item, pg.InfiniteLine) and item != self.depth_line: self.waterfall.removeItem(item)
        for idx in tick_indices[1:]:
            line = pg.InfiniteLine(pos=idx, angle=0, pen=pg.mkPen(color=(255,255,255,30), style=Qt.DashLine)); self.waterfall.addItem(line)

    def waterfall_plot_callback(self, spectrogram, depth_index, drive_frequency, override_idx):
        filtered = sonar_display_pipeline(spectrogram)
        self.data = np.roll(self.data, -1, axis=0); self.data[-1, :] = filtered
        self.imageitem.setImage(self.data.T, autoLevels=False); self.imageitem.setLevels((10, 220))
        depth_m, ovr_m = (depth_index * SAMPLE_RESOLUTION) / 100.0, (override_idx * SAMPLE_RESOLUTION) / 100.0
        
        if depth_index < 20 or depth_index >= NUM_SAMPLES:
            is_valid = False
        else:
            is_valid = abs(depth_index - override_idx) <= INDEX_TOLERANCE

        # [關鍵修正] 無論是否顯示線條，都要在背景更新歷史數據！
        # 這樣當使用者重新開啟線條時，數據才是同步的
        target_depth_idx = depth_index if self.depth_line_mode == "Auto" else override_idx
        
        # 處理無效數據為 NaN
        val_to_plot = target_depth_idx if is_valid else np.nan
        
        if self.depth_line_mode == "Override" and target_depth_idx >= 20:
            val_to_plot = target_depth_idx

        # 滾動歷史數據
        self.depth_history = np.roll(self.depth_history, -1)
        self.depth_history[-1] = val_to_plot

        if self.show_depth_line:
            self.depth_line.setData(
                x=np.arange(MAX_ROWS), 
                y=self.depth_history, 
                connect="finite"
            )
            # [修改] 寬度設為 6
            self.depth_line.setPen(pg.mkPen(color='k', width=6))
            self.depth_line.show()
        else:
            self.depth_line.hide()

        if self.large_depth_visible:
            if is_valid:
                color_hex = "#FFFFFF" 
            else:
                color_hex = "rgba(255, 255, 255, 0.2)" 
            
            freq_text = f"&nbsp;&nbsp;{drive_frequency:.0f}kHz"
            
            html_str = f"""
            <div style="text-align: left; line-height: 90%; font-family: 'Malgun Gothic';">
                <span style="font-size: 64pt; font-weight: 600; color: {color_hex};">{depth_m:.1f}</span>
                <span style="font-size: 32pt; font-weight: 600; color: {color_hex};">m</span><br>
                <span style="font-size: 14pt; color: #cccccc; font-weight: 600;">{freq_text}</span>
            </div>
            """
            
            overlay_pos = self.current_zoom_samples - (self.current_zoom_samples * 0.02)
            self.depth_overlay.setPos(10, overlay_pos)
            self.depth_overlay.setHtml(html_str)

        self.lbl_depth.setText(f"D: {depth_m:.1f}m")
        self.lbl_ovr.setText(f"O: {ovr_m:.1f}m")

    def set_sound_speed(self, speed):
        global SPEED_OF_SOUND, SAMPLE_RESOLUTION, MAX_DEPTH, depth_labels
        SPEED_OF_SOUND = self.current_speed = speed
        SAMPLE_RESOLUTION = (SPEED_OF_SOUND * SAMPLE_TIME * 100) / 2
        MAX_DEPTH = NUM_SAMPLES * SAMPLE_RESOLUTION
        ax = self.waterfall.getAxis("right"); ax.setTickFont(QFont("Arial", 10))
        self.update_zoom_range()

    def handle_main_connect(self):
        if self.is_connected:
            if self.serial_thread: self.serial_thread.stop(); self.serial_thread = None
            if self.udp_thread: self.udp_thread.stop(); self.udp_thread = None
            self.is_connected = False; self.btn_connect.setText("Connect"); self.btn_connect.setProperty("class", "sidebar_btn"); self.btn_connect.setStyle(self.btn_connect.style()) 
            self.lbl_depth.setText("Dep: ---"); self.lbl_ovr.setText("Ovr: ---")
        else:
            if self.connection_source == "Serial Port":
                if not self.serial_port_name or self.serial_port_name == "No Ports": return print("No Serial Port Selected")
                self.serial_thread = SerialReader(self.serial_port_name, BAUD_RATE); self.serial_thread.data_received.connect(self.waterfall_plot_callback); self.serial_thread.start()
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
