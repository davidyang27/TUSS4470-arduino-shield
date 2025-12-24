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
    QDialog, QFormLayout, QFrame
)
from PyQt5.QtCore import QThread, pyqtSignal, Qt
from PyQt5.QtGui import QPalette, QColor, QFont
import pyqtgraph as pg
import qdarktheme

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
PACKET_SIZE = 1 + 6 + NUM_SAMPLES + 1  # header + payload + checksum
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
    return [port.device for port in serial.tools.list_ports.comports()][::-1]

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
                    data, _ = sock.recvfrom(PACKET_SIZE + 100)
                    if data[0] == 0xAA:
                        payload = data[1:1+6+NUM_SAMPLES]
                        d, f, v = struct.unpack("<HhH", payload[:6])
                        self.data_received.emit(np.frombuffer(payload[6:], dtype=np.uint8), d, f, float(v))
                except: continue
        finally: sock.close()

# --- 設定對話框 ---
class SettingsDialog(QDialog):
    def __init__(self, parent=None, current_speed=343, current_gradient="cyclic", nmea_enabled=False, nmea_port=10110, ip="127.0.0.1"):
        super().__init__(parent)
        self.main_app = parent
        self.setWindowTitle("Chart Settings")
        self.setFixedSize(380, 680)

        main_layout = QVBoxLayout(self)
        main_layout.addWidget(QLabel("<b>Environment & Appearance</b>"))
        app_group = QWidget()
        app_layout = QFormLayout(app_group)
        self.speed_dropdown = QComboBox()
        self.speed_dropdown.addItems([f"{AIR_SPEED} m/s (Air)", f"{WATER_SPEED} m/s (Water)"])
        self.speed_dropdown.setCurrentIndex(1 if current_speed == WATER_SPEED else 0)
        self.gradient_dropdown = QComboBox()
        self.gradient_dropdown.addItems(["viridis", "plasma", "inferno", "magma", "thermal", "flame", "yellowy", "bipolar", "spectrum", "cyclic", "greyclip", "grey"])
        self.gradient_dropdown.setCurrentText(current_gradient)
        app_layout.addRow("Operation Mode:", self.speed_dropdown)
        app_layout.addRow("Color Map:", self.gradient_dropdown)
        main_layout.addWidget(app_group)

        main_layout.addWidget(QLabel("<b>Display Options</b>"))
        disp_group = QWidget()
        disp_layout = QFormLayout(disp_group)
        self.large_depth_checkbox = QCheckBox("Enable Depth Overlay")
        self.large_depth_checkbox.setChecked(self.main_app.large_depth_visible)
        self.overlay_mode_combo = QComboBox()
        self.overlay_mode_combo.addItems(["Auto (Threshold)", "Override (Max)"])
        self.overlay_mode_combo.setCurrentIndex(0 if self.main_app.depth_overlay_mode == "Auto" else 1)
        self.show_line_checkbox = QCheckBox("Show Red Depth Line")
        self.show_line_checkbox.setChecked(self.main_app.show_depth_line)
        self.line_mode_combo = QComboBox()
        self.line_mode_combo.addItems(["Follow Auto", "Follow Override"])
        self.line_mode_combo.setCurrentIndex(0 if self.main_app.depth_line_mode == "Auto" else 1)
        disp_layout.addRow(self.large_depth_checkbox)
        disp_layout.addRow("Overlay Source:", self.overlay_mode_combo)
        disp_layout.addRow(self.show_line_checkbox)
        disp_layout.addRow("Line Follow Mode:", self.line_mode_combo)
        main_layout.addWidget(disp_group)

        main_layout.addWidget(QLabel("<b>Register Write (Hex)</b>"))
        reg_group = QWidget()
        reg_layout = QHBoxLayout(reg_group)
        self.reg_input = QLineEdit(); self.reg_input.setPlaceholderText("Addr, Data")
        self.reg_btn = QPushButton("Send"); self.reg_btn.clicked.connect(self.handle_reg_send)
        reg_layout.addWidget(self.reg_input); reg_layout.addWidget(self.reg_btn)
        main_layout.addWidget(reg_group)

        main_layout.addWidget(QLabel("<b>NMEA Output</b>"))
        nmea_group = QWidget()
        nmea_layout = QFormLayout(nmea_group)
        self.nmea_checkbox = QCheckBox("Enable NMEA TCP Output"); self.nmea_checkbox.setChecked(nmea_enabled)
        self.port_input = QLineEdit(str(nmea_port))
        nmea_layout.addRow("Enable:", self.nmea_checkbox); nmea_layout.addRow("TCP Port:", self.port_input)
        main_layout.addWidget(nmea_group)

        btn_layout = QHBoxLayout()
        apply_btn = QPushButton("Apply"); apply_btn.clicked.connect(self.handle_apply)
        btn_layout.addStretch(); btn_layout.addWidget(apply_btn); btn_layout.addWidget(QPushButton("Cancel", clicked=self.close))
        main_layout.addStretch(); main_layout.addLayout(btn_layout)
        self.setStyleSheet("QWidget { background-color: #2b2b2b; color: white; } QComboBox, QLineEdit { background-color: #3c3c3c; padding: 4px; } QPushButton { background-color: #444; padding: 6px 14px; border-radius: 4px; }")

    def handle_reg_send(self):
        txt = self.reg_input.text().strip()
        if ',' in txt:
            try:
                addr, data = [int(x.strip(), 16) for x in txt.split(',')]
                if self.main_app.serial_thread: self.main_app.serial_thread.send_raw_command(struct.pack('BBB', ord('W'), addr, data))
            except: pass

    def handle_apply(self):
        if self.main_app:
            speed = AIR_SPEED if self.speed_dropdown.currentIndex() == 0 else WATER_SPEED
            self.main_app.set_sound_speed(speed)
            self.main_app.set_gradient(self.gradient_dropdown.currentText())
            self.main_app.large_depth_visible = self.large_depth_checkbox.isChecked()
            self.main_app.depth_overlay.setVisible(self.main_app.large_depth_visible)
            self.main_app.depth_overlay_mode = "Auto" if self.overlay_mode_combo.currentIndex() == 0 else "Override"
            self.main_app.show_depth_line = self.show_line_checkbox.isChecked()
            self.main_app.depth_line_mode = "Auto" if self.line_mode_combo.currentIndex() == 0 else "Override"
            if not self.main_app.show_depth_line: self.main_app.depth_line.hide()
            port = int(self.port_input.text()) if self.port_input.text().isdigit() else 10110
            self.main_app.configure_nmea_output(self.nmea_checkbox.isChecked(), port)
        self.close()

# --- 主應用程式 ---
class WaterfallApp(QMainWindow):
    def __init__(self):
        super().__init__()
        self.serial_thread = None; self.udp_thread = None
        self.nmea_output_enabled = False; self.nmea_port = 10110
        self.large_depth_visible = True; self.depth_overlay_mode = "Auto"
        self.show_depth_line = True; self.depth_line_mode = "Auto"
        self.current_gradient = 'cyclic'
        self.current_speed = SPEED_OF_SOUND 

        self.setWindowTitle("Open Echo Interface")
        self.setGeometry(0, 0, 850, 850)
        self.data = np.zeros((MAX_ROWS, NUM_SAMPLES))

        central = QWidget(); self.setCentralWidget(central)
        main_layout = QVBoxLayout(central); main_layout.setContentsMargins(15, 10, 15, 10); main_layout.setSpacing(10)

        row1 = QHBoxLayout(); font1 = QFont("Arial", 10)
        row1.addWidget(QLabel("UDP Port:"))
        self.udp_input = QLineEdit("5005"); self.udp_input.setFixedWidth(60); self.udp_input.setFont(font1); row1.addWidget(self.udp_input)
        self.udp_conn_btn = QPushButton("Connect UDP"); self.udp_conn_btn.setFont(font1); self.udp_conn_btn.clicked.connect(self.toggle_udp_connection); row1.addWidget(self.udp_conn_btn)
        row1.addStretch()
        row1.addWidget(QLabel("Port:"))
        self.serial_combo = QComboBox(); self.serial_combo.setFixedWidth(120); self.serial_combo.setFont(font1); self.serial_combo.addItems(get_serial_ports()); row1.addWidget(self.serial_combo)
        self.conn_btn = QPushButton("Connect"); self.conn_btn.setFont(font1); self.conn_btn.clicked.connect(self.toggle_serial_connection); row1.addWidget(self.conn_btn)
        main_layout.addLayout(row1)

        # ============================================================
        # [ Row 2 ] Echogram 調整 - 徹底內置 Y 軸
        # ============================================================
        self.waterfall = pg.PlotWidget()
        
        # 修正 1：徹底移除 ViewBox 邊距
        vb = self.waterfall.getViewBox()
        vb.setDefaultPadding(0)
        
        # 修正 2：將 Y 軸設定為右側，並限制其佔用空間
        self.waterfall.showAxis('right')
        self.waterfall.hideAxis('left')
        self.waterfall.hideAxis('bottom') # 隱藏 X 軸
        
        y_right = self.waterfall.getAxis('right')
        y_right.setStyle(showValues=True, tickLength=-10) # 刻度朝內指向圖內
        y_right.setZValue(10) # 讓數字層級高於影像，避免被遮擋
        y_right.setWidth(40)  # 設定固定寬度以解決 Y 軸消失問題
        
        self.imageitem = pg.ImageItem(axisOrder="row-major")
        self.waterfall.addItem(self.imageitem)
        self.waterfall.invertY(True)
        
        # 深度大字
        self.depth_overlay = pg.TextItem(text="--- m", color=(255, 255, 255), anchor=(0, 1))
        self.depth_overlay.setFont(QFont("Arial", 60, QFont.Bold))
        self.depth_overlay.setZValue(100) # 確保大字在最最上層
        self.waterfall.addItem(self.depth_overlay)
        
        self.set_sound_speed(self.current_speed) 
        
        self.depth_line = pg.InfiniteLine(angle=0, pen=pg.mkPen("r", width=2))
        self.depth_line.setZValue(50) # 讓紅線在影像上方，文字下方
        self.waterfall.addItem(self.depth_line)
        
        main_layout.addWidget(self.waterfall, stretch=1)

        # Row 4 & 5 其他不變
        row4 = QHBoxLayout(); info_style = "font-size: 18px; color: white; font-weight: bold;"
        self.depth_lbl = QLabel("Depth: --- cm"); self.freq_lbl = QLabel("Freq: --- kHz"); self.ovr_lbl = QLabel("Override: ---")
        for lbl in [self.depth_lbl, self.freq_lbl, self.ovr_lbl]: lbl.setStyleSheet(info_style); row4.addWidget(lbl)
        row4.addStretch(); main_layout.addLayout(row4)

        row5 = QHBoxLayout(); font5 = QFont("Arial", 10)
        self.set_btn = QPushButton("Settings"); self.set_btn.setFont(font5); self.set_btn.clicked.connect(self.open_settings); row5.addWidget(self.set_btn)
        row5.addStretch()
        self.quit_btn = QPushButton("Quit"); self.quit_btn.setFont(font5); self.quit_btn.clicked.connect(self.close); row5.addWidget(self.quit_btn)
        main_layout.addLayout(row5)

        self.colorbar = pg.HistogramLUTWidget(); self.colorbar.setImageItem(self.imageitem)
        self.colorbar.item.gradient.loadPreset("cyclic"); self.imageitem.setLevels(DEFAULT_LEVELS)

    def waterfall_plot_callback(self, spectrogram, depth_index, drive_frequency, override_idx):
        filtered = sonar_display_pipeline(spectrogram)
        self.data = np.roll(self.data, -1, axis=0); self.data[-1, :] = filtered
        self.imageitem.setImage(self.data.T, autoLevels=False); self.imageitem.setLevels((10, 220))
        
        depth_m = (depth_index * SAMPLE_RESOLUTION) / 100.0
        ovr_m = (override_idx * SAMPLE_RESOLUTION) / 100.0
        is_valid = abs(depth_index - override_idx) <= INDEX_TOLERANCE

        if self.show_depth_line:
            target = depth_index if self.depth_line_mode == "Auto" else override_idx
            self.depth_line.setPos(target); self.depth_line.show()
            self.depth_line.setPen(pg.mkPen((255,0,0,120 if (self.depth_line_mode=="Auto" and not is_valid) else 255), width=3))
        else: self.depth_line.hide()

        if self.large_depth_visible:
            self.depth_overlay.setPos(10, NUM_SAMPLES)
            val = depth_m if self.depth_overlay_mode == "Auto" else ovr_m
            self.depth_overlay.setColor(QColor(255, 255, 255) if is_valid else QColor(255, 100, 100))
            self.depth_overlay.setText(f"{val:.1f} m" if (is_valid or self.depth_overlay_mode=="Override") else "0.0 m")

        self.depth_lbl.setText(f"Depth: {depth_m*100:.1f} cm"); self.freq_lbl.setText(f"Freq: {drive_frequency:.1f} kHz"); self.ovr_lbl.setText(f"Override: {ovr_m*100:.1f} cm")

    def set_sound_speed(self, speed):
        global SPEED_OF_SOUND, SAMPLE_RESOLUTION, MAX_DEPTH, depth_labels
        SPEED_OF_SOUND = self.current_speed = speed
        SAMPLE_RESOLUTION = (SPEED_OF_SOUND * SAMPLE_TIME * 100) / 2
        MAX_DEPTH = NUM_SAMPLES * SAMPLE_RESOLUTION
        depth_labels = {int(i / SAMPLE_RESOLUTION): f"{i / 100}" for i in range(0, int(MAX_DEPTH), Y_LABEL_DISTANCE)}
        
        tick_font = QFont("Arial", 9) 
        ax = self.waterfall.getAxis("right")
        ax.setTickFont(tick_font)
        ax.setTicks([list(depth_labels.items())[::-1]])
        
        # 重新繪製背景點線
        for item in self.waterfall.items():
            if isinstance(item, pg.InfiniteLine) and item != self.depth_line: self.waterfall.removeItem(item)
        for i in range(0, int(MAX_DEPTH), Y_LABEL_DISTANCE):
            self.waterfall.addItem(pg.InfiniteLine(pos=int(i/SAMPLE_RESOLUTION), angle=0, pen=pg.mkPen(color=(255,255,255,40), style=Qt.DotLine)))

    def toggle_serial_connection(self):
        if self.serial_thread and self.serial_thread.isRunning(): self.serial_thread.stop(); self.conn_btn.setText("Connect")
        else:
            self.serial_thread = SerialReader(self.serial_combo.currentText(), BAUD_RATE)
            self.serial_thread.data_received.connect(self.waterfall_plot_callback); self.serial_thread.start(); self.conn_btn.setText("Disconnect")

    def toggle_udp_connection(self):
        if hasattr(self, 'udp_thread') and self.udp_thread: self.udp_thread.stop(); self.udp_thread = None; self.udp_conn_btn.setText("Connect UDP")
        else:
            self.udp_thread = UDPReader(int(self.udp_input.text()))
            self.udp_thread.data_received.connect(self.waterfall_plot_callback); self.udp_thread.start(); self.udp_conn_btn.setText("Disconnect UDP")

    def open_settings(self):
        dlg = SettingsDialog(self, self.current_speed, self.current_gradient, self.nmea_output_enabled, self.nmea_port, get_local_ip())
        dlg.setWindowModality(Qt.ApplicationModal); dlg.exec_()

    def set_gradient(self, n): self.current_gradient = n; self.colorbar.item.gradient.loadPreset(n)
    def configure_nmea_output(self, e, p): self.nmea_output_enabled, self.nmea_port = e, p
    def closeEvent(self, e):
        if self.serial_thread: self.serial_thread.stop()
        if hasattr(self, 'udp_thread') and self.udp_thread: self.udp_thread.stop()
        e.accept()

if __name__ == "__main__":
    app = QApplication(sys.argv); qdarktheme.setup_theme("dark")
    window = WaterfallApp(); window.show()
    sys.exit(app.exec())
