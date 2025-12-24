import sys
import numpy as np
import serial
import serial.tools.list_ports
import struct
import time
import socket
import queue
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QVBoxLayout, QWidget, QComboBox, 
    QPushButton, QLabel, QLineEdit, QHBoxLayout, QCheckBox, 
    QDialog, QFormLayout, QFrame, QSizePolicy, QGroupBox
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
depth_labels = {int(i / SAMPLE_RESOLUTION): f"{i / 100}" for i in range(0, int(MAX_DEPTH), Y_LABEL_DISTANCE)}

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
        self.setWindowTitle("Configuration")
        self.setFixedSize(400, 750)
        # 專業深色主題樣式表
        self.setStyleSheet("""
            QDialog { background-color: #2b2b2b; color: #e0e0e0; font-family: Arial; }
            QLabel { color: #e0e0e0; font-weight: bold; font-size: 14px; }
            QComboBox, QLineEdit { 
                background-color: #3a3a3a; 
                border: 1px solid #555; 
                color: white; 
                padding: 6px; 
                border-radius: 3px;
            }
            QComboBox::drop-down { border: none; background: #444; }
            QPushButton { 
                background-color: #444; 
                border: 1px solid #666; 
                color: white; 
                padding: 10px; 
                border-radius: 4px; font-weight: bold;
            }
            QPushButton:hover { background-color: #555; border-color: #777; }
            QPushButton:pressed { background-color: #0078d7; border-color: #005a9e; }
            QGroupBox { 
                border: 1px solid #555; 
                margin-top: 12px; 
                font-weight: bold; 
                color: #0078d7; /* 標題藍色跳色 */
                padding-top: 15px;
            }
            QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 5px; background-color: #2b2b2b; }
        """)

        layout = QVBoxLayout(self)

        # 1. 連線設定
        conn_group = QGroupBox("Connection Source")
        conn_layout = QFormLayout(conn_group)
        self.source_combo = QComboBox(); self.source_combo.addItems(["Serial Port", "UDP Stream"]); self.source_combo.setCurrentText(self.main_app.connection_source)
        self.serial_combo = QComboBox(); self.serial_combo.addItems(get_serial_ports()); self.serial_combo.setCurrentText(self.main_app.serial_port_name)
        self.udp_port_input = QLineEdit(str(self.main_app.udp_port_num))
        conn_layout.addRow("Source Type:", self.source_combo); conn_layout.addRow("Serial Port:", self.serial_combo); conn_layout.addRow("UDP Port:", self.udp_port_input)
        self.source_combo.currentTextChanged.connect(self.update_inputs); self.update_inputs(self.source_combo.currentText())
        layout.addWidget(conn_group)

        # 2. 環境與外觀
        app_group = QGroupBox("Environment & Display")
        app_layout = QFormLayout(app_group)
        self.speed_dropdown = QComboBox(); self.speed_dropdown.addItems([f"{AIR_SPEED} m/s (Air)", f"{WATER_SPEED} m/s (Water)"]); self.speed_dropdown.setCurrentIndex(1 if self.main_app.current_speed == WATER_SPEED else 0)
        self.gradient_dropdown = QComboBox(); self.gradient_dropdown.addItems(["viridis", "plasma", "inferno", "magma", "thermal", "flame", "yellowy", "bipolar", "spectrum", "cyclic", "greyclip", "grey"]); self.gradient_dropdown.setCurrentText(self.main_app.current_gradient)
        app_layout.addRow("Sound Speed:", self.speed_dropdown); app_layout.addRow("Color Map:", self.gradient_dropdown)
        layout.addWidget(app_group)

        # 3. 顯示選項
        disp_group = QGroupBox("Overlays")
        disp_layout = QFormLayout(disp_group)
        self.large_depth_checkbox = QCheckBox("Show Depth Number"); self.large_depth_checkbox.setChecked(self.main_app.large_depth_visible)
        self.overlay_mode_combo = QComboBox(); self.overlay_mode_combo.addItems(["Auto (Threshold)", "Override (Max)"]); self.overlay_mode_combo.setCurrentIndex(0 if self.main_app.depth_overlay_mode == "Auto" else 1)
        self.show_line_checkbox = QCheckBox("Show Red Depth Line"); self.show_line_checkbox.setChecked(self.main_app.show_depth_line)
        self.line_mode_combo = QComboBox(); self.line_mode_combo.addItems(["Follow Auto", "Follow Override"]); self.line_mode_combo.setCurrentIndex(0 if self.main_app.depth_line_mode == "Auto" else 1)
        disp_layout.addRow(self.large_depth_checkbox); disp_layout.addRow("Number Source:", self.overlay_mode_combo); disp_layout.addRow(self.show_line_checkbox); disp_layout.addRow("Line Follow:", self.line_mode_combo)
        layout.addWidget(disp_group)

        # 4. NMEA 輸出
        nmea_group = QGroupBox("NMEA Output")
        nmea_layout = QFormLayout(nmea_group)
        self.nmea_checkbox = QCheckBox("Enable NMEA TCP"); self.nmea_checkbox.setChecked(self.main_app.nmea_output_enabled)
        self.nmea_port_input = QLineEdit(str(self.main_app.nmea_port))
        nmea_layout.addRow("Enable:", self.nmea_checkbox); nmea_layout.addRow("TCP Port:", self.nmea_port_input)
        layout.addWidget(nmea_group)

        # 5. [功能保留] Register Write (Hex Send)
        reg_group = QGroupBox("Device Register (Hex)")
        reg_layout = QHBoxLayout(reg_group)
        self.reg_input = QLineEdit(); self.reg_input.setPlaceholderText("Addr, Data (e.g. 13, 06)")
        self.reg_btn = QPushButton("Send"); self.reg_btn.clicked.connect(self.handle_reg_send)
        reg_layout.addWidget(self.reg_input); reg_layout.addWidget(self.reg_btn)
        layout.addWidget(reg_group)

        # Buttons
        btn_layout = QHBoxLayout()
        apply_btn = QPushButton("Apply Settings"); apply_btn.clicked.connect(self.handle_apply)
        # Apply 按鈕特別使用藍色凸顯
        apply_btn.setStyleSheet("background-color: #0078d7; border: 1px solid #005a9e;") 
        cancel_btn = QPushButton("Cancel"); cancel_btn.clicked.connect(self.close)
        btn_layout.addStretch(); btn_layout.addWidget(apply_btn); btn_layout.addWidget(cancel_btn)
        layout.addLayout(btn_layout)

    def update_inputs(self, text):
        is_serial = (text == "Serial Port")
        self.serial_combo.setEnabled(is_serial)
        self.udp_port_input.setEnabled(not is_serial)

    def handle_reg_send(self):
        # 這裡就是發送 Hex 的邏輯，功能已保留
        txt = self.reg_input.text().strip()
        if ',' in txt:
            try:
                addr, data = [int(x.strip(), 16) for x in txt.split(',')]
                # 只有當 Serial Thread 存在且運行中時才發送
                if self.main_app.serial_thread and self.main_app.serial_thread.isRunning():
                    self.main_app.serial_thread.send_raw_command(struct.pack('BBB', ord('W'), addr, data))
                    print(f"Sent Register: Addr 0x{addr:02X}, Data 0x{data:02X}")
                else:
                    print("Serial not connected, cannot send register.")
            except Exception as e: print(f"Hex Error: {e}")

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

        self.setWindowTitle("Open Echo Interface")
        self.setGeometry(0, 0, 1100, 700)
        
        # ============================================================
        # [核心] SIMRAD 風格專業樣式表 (QSS)
        # ============================================================
        self.setStyleSheet("""
            /* 全局設定：Arial 字體，深色背景 */
            * { font-family: Arial, sans-serif; }
            QMainWindow { background-color: black; } 
            QWidget { background-color: black; color: #e0e0e0; }
            
            /* 側邊欄框架：深炭灰色 */
            QFrame#sidebarFrame {
                background-color: #2b2b2b; 
                border-left: 2px solid #1a1a1a;
            }

            /* 側邊欄按鈕樣式 */
            QPushButton.sidebar_btn {
                background-color: #3a3a3a; /* 基礎深灰 */
                border: 1px solid #555555; 
                color: white;
                font-size: 20px;
                font-weight: bold;
                border-radius: 2px; 
                padding: 10px;
                background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #444, stop:1 #333);
            }
            /* 滑鼠懸停 */
            QPushButton.sidebar_btn:hover {
                background-color: #4a4a4a;
                border-color: #888888;
                background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #555, stop:1 #444);
            }
            /* 按下狀態 */
            QPushButton.sidebar_btn:pressed {
                background-color: #1a1a1a;
                border-color: #0078d7;
                color: #0078d7;
            }

            /* 連線中 (Stop) 按鈕：深紅色 */
            QPushButton.connect_active {
                background-color: #8b0000;
                border: 2px solid #ff3333;
                color: white;
                font-size: 20px; font-weight: bold; padding: 10px;
                background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #a00000, stop:1 #700000);
            }
            QPushButton.connect_active:hover { background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #c00000, stop:1 #900000); }
        """)

        self.data = np.zeros((MAX_ROWS, NUM_SAMPLES))
        central = QWidget(); self.setCentralWidget(central)
        main_layout = QHBoxLayout(central); main_layout.setContentsMargins(0, 0, 0, 0); main_layout.setSpacing(0)

        # ------------------ 左側：聲納圖 ------------------
        self.waterfall = pg.PlotWidget(background='k')
        vb = self.waterfall.getViewBox(); vb.setDefaultPadding(0)
        self.waterfall.showAxis('right'); self.waterfall.hideAxis('left'); self.waterfall.hideAxis('bottom')
        
        # Y 軸設定：外部白色數字，確保不消失
        y_right = self.waterfall.getAxis('right')
        y_right.setWidth(60) 
        y_right.setStyle(showValues=True)
        y_right.setTextPen(pg.mkPen(color='w'))
        y_right.setPen(pg.mkPen(color=(100,100,100)))

        self.imageitem = pg.ImageItem(axisOrder="row-major"); self.waterfall.addItem(self.imageitem); self.waterfall.invertY(True)
        
        # 深度大字
        self.depth_overlay = pg.TextItem(text="--- m", color=(255, 255, 255), anchor=(0, 1))
        self.depth_overlay.setFont(QFont("Arial Black", 64, QFont.Bold))
        self.depth_overlay.setZValue(200); self.waterfall.addItem(self.depth_overlay)
        
        # 狀態文字
        self.status_text = pg.TextItem(text="Ready", color=(200, 200, 200), anchor=(0, 0))
        self.status_text.setFont(QFont("Arial", 14, QFont.Bold))
        self.status_text.setPos(10, 10); self.waterfall.addItem(self.status_text)

        self.set_sound_speed(self.current_speed)
        self.depth_line = pg.InfiniteLine(angle=0, pen=pg.mkPen("r", width=2)); self.depth_line.setZValue(50); self.waterfall.addItem(self.depth_line)
        
        main_layout.addWidget(self.waterfall, stretch=1)

        # ------------------ 右側：功能按鈕列 ------------------
        sidebar = QFrame()
        sidebar.setObjectName("sidebarFrame") 
        sidebar.setFixedWidth(130)
        side_layout = QVBoxLayout(sidebar); side_layout.setContentsMargins(5, 5, 5, 5); side_layout.setSpacing(5)

        self.btn_plus = QPushButton("+"); self.btn_plus.setProperty("class", "sidebar_btn"); self.btn_plus.setFixedHeight(80); side_layout.addWidget(self.btn_plus)
        self.btn_minus = QPushButton("-"); self.btn_minus.setProperty("class", "sidebar_btn"); self.btn_minus.setFixedHeight(80); side_layout.addWidget(self.btn_minus)
        
        spacer = QWidget(); spacer.setSizePolicy(QSizePolicy.Minimum, QSizePolicy.Expanding); spacer.setStyleSheet("background-color: transparent;"); side_layout.addWidget(spacer)

        self.btn_settings = QPushButton("Settings"); self.btn_settings.setProperty("class", "sidebar_btn"); self.btn_settings.setFixedHeight(80); self.btn_settings.clicked.connect(self.open_settings); side_layout.addWidget(self.btn_settings)
        self.btn_connect = QPushButton("Connect"); self.btn_connect.setProperty("class", "sidebar_btn"); self.btn_connect.setFixedHeight(80); self.btn_connect.clicked.connect(self.handle_main_connect); side_layout.addWidget(self.btn_connect)

        main_layout.addWidget(sidebar)

        self.colorbar = pg.HistogramLUTWidget(); self.colorbar.setImageItem(self.imageitem); self.colorbar.item.gradient.loadPreset("cyclic"); self.imageitem.setLevels(DEFAULT_LEVELS)

    def waterfall_plot_callback(self, spectrogram, depth_index, drive_frequency, override_idx):
        filtered = sonar_display_pipeline(spectrogram)
        self.data = np.roll(self.data, -1, axis=0); self.data[-1, :] = filtered
        self.imageitem.setImage(self.data.T, autoLevels=False); self.imageitem.setLevels((10, 220))
        depth_m, ovr_m = (depth_index * SAMPLE_RESOLUTION) / 100.0, (override_idx * SAMPLE_RESOLUTION) / 100.0
        is_valid = abs(depth_index - override_idx) <= INDEX_TOLERANCE

        if self.show_depth_line:
            target = depth_index if self.depth_line_mode == "Auto" else override_idx
            self.depth_line.setPos(target); self.depth_line.show()
            self.depth_line.setPen(pg.mkPen((255,0,0,120 if (self.depth_line_mode=="Auto" and not is_valid) else 255), width=3))
        else: self.depth_line.hide()

        if self.large_depth_visible:
            self.depth_overlay.setPos(10, NUM_SAMPLES); val = depth_m if self.depth_overlay_mode == "Auto" else ovr_m
            self.depth_overlay.setColor(QColor(255, 255, 255) if is_valid else QColor(255, 100, 100))
            self.depth_overlay.setText(f"{val:.1f} m" if (is_valid or self.depth_overlay_mode=="Override") else "0.0 m")

        self.status_text.setText(f"Freq: {drive_frequency:.1f} kHz | Depth: {depth_m*100:.1f} cm")

    def set_sound_speed(self, speed):
        global SPEED_OF_SOUND, SAMPLE_RESOLUTION, MAX_DEPTH, depth_labels
        SPEED_OF_SOUND = self.current_speed = speed
        SAMPLE_RESOLUTION = (SPEED_OF_SOUND * SAMPLE_TIME * 100) / 2
        MAX_DEPTH = NUM_SAMPLES * SAMPLE_RESOLUTION
        depth_labels = {int(i / SAMPLE_RESOLUTION): f"{i / 100}" for i in range(0, int(MAX_DEPTH), Y_LABEL_DISTANCE)}
        ax = self.waterfall.getAxis("right"); ax.setTickFont(QFont("Arial", 10)); ax.setTicks([list(depth_labels.items())[::-1]])
        for item in self.waterfall.items():
            if isinstance(item, pg.InfiniteLine) and item != self.depth_line: self.waterfall.removeItem(item)
        for i in range(0, int(MAX_DEPTH), Y_LABEL_DISTANCE):
            self.waterfall.addItem(pg.InfiniteLine(pos=int(i/SAMPLE_RESOLUTION), angle=0, pen=pg.mkPen(color=(255,255,255,40), style=Qt.DotLine)))

    def handle_main_connect(self):
        if self.is_connected:
            if self.serial_thread: self.serial_thread.stop(); self.serial_thread = None
            if self.udp_thread: self.udp_thread.stop(); self.udp_thread = None
            self.is_connected = False; self.btn_connect.setText("Connect"); self.btn_connect.setProperty("class", "sidebar_btn"); self.btn_connect.setStyle(self.btn_connect.style()) 
            self.status_text.setText("Disconnected")
        else:
            if self.connection_source == "Serial Port":
                if not self.serial_port_name or self.serial_port_name == "No Ports": return print("No Serial Port Selected")
                self.serial_thread = SerialReader(self.serial_port_name, BAUD_RATE); self.serial_thread.data_received.connect(self.waterfall_plot_callback); self.serial_thread.start()
            else:
                try: self.udp_thread = UDPReader(self.udp_port_num); self.udp_thread.data_received.connect(self.waterfall_plot_callback); self.udp_thread.start()
                except: return
            self.is_connected = True; self.btn_connect.setText("Stop"); self.btn_connect.setProperty("class", "connect_active"); self.btn_connect.setStyle(self.btn_connect.style())
            self.status_text.setText(f"Connected: {self.connection_source}")

    def open_settings(self): dlg = SettingsDialog(self); dlg.exec_()
    def set_gradient(self, n): self.current_gradient = n; self.colorbar.item.gradient.loadPreset(n)
    def configure_nmea_output(self, e, p): self.nmea_output_enabled, self.nmea_port = e, p
    def closeEvent(self, e):
        if self.serial_thread: self.serial_thread.stop()
        if self.udp_thread: self.udp_thread.stop()
        e.accept()

if __name__ == "__main__":
    app = QApplication(sys.argv); window = WaterfallApp(); window.show(); sys.exit(app.exec())
