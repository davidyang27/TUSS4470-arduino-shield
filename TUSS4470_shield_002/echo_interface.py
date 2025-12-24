import sys
import numpy as np
import serial
import serial.tools.list_ports
import struct
import time
import socket
from PyQt5.QtWidgets import (
    QApplication,
    QMainWindow,
    QVBoxLayout,
    QWidget,
    QComboBox,
    QPushButton,
    QLabel,
    QLineEdit,
    QFormLayout,
    QDialog,
    QFrame,
)
from PyQt5.QtCore import QThread, pyqtSignal
import pyqtgraph as pg
import qdarktheme
from PyQt5.QtWidgets import (
    QHBoxLayout,
)
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QPalette, QColor, QFont
from PyQt5.QtWidgets import QVBoxLayout, QLabel, QCheckBox, QLineEdit
from PyQt5.QtWidgets import QApplication
import queue

# --- Configuration Parameters ---
BAUD_RATE = 250000
NUM_SAMPLES = 1800
INDEX_TOLERANCE = 10  # 容許誤差範圍 (Sample Index)
# --------------------------------

MAX_ROWS = 300  # Number of time steps (Y-axis)
Y_LABEL_DISTANCE = 50  # distance between labels in cm

# SPEED_OF_SOUND = 1440  # default sound speed meters/second in water
SPEED_OF_SOUND = 343  # default sound speed meters/second in water

# SAMPLE_TIME = 52.226e-6  # 13.2 microseconds on Atmega328 max sample speed plus 50 microseconds delay in sampling loop
# SAMPLE_TIME = 47.0e-6
# SAMPLE_TIME = 41.666e-6 # 13.2 microseconds on Atmega328 max sample speed plus 40 microseconds delay in sampling loop
# SAMPLE_TIME = 22.22e-6  # 13.2 microseconds on Atmega328 max sample speed plus 20 microseconds delay in sampling loop
SAMPLE_TIME = 13.2e-6     # 13.2 microseconds on Atmega328 max sample speed without additional delay
# SAMPLE_TIME = 11.0e-6     # 13.2 microseconds on RP2040 max sample speed with 10 microseconds additional delay per sample
# SAMPLE_TIME = 7.682e-6  # 7.682 microseconds on STM32F103 max sample speed
# SAMPLE_TIME = 6.0e-6  # 6 microseconds on RP2040 max sample speed with 5 microseconds additional delay per sample
# SAMPLE_TIME = 1.290e-6     # 13.2 microseconds on RP2040 max sample speed without additional delay

DEFAULT_LEVELS = (0, 256)  # Expected data range

SAMPLE_RESOLUTION = (SPEED_OF_SOUND * SAMPLE_TIME * 100) / 2  # cm per row (0.99 cm per row)
PACKET_SIZE = 1 + 6 + NUM_SAMPLES + 1  # header + payload + checksum
MAX_DEPTH = NUM_SAMPLES * SAMPLE_RESOLUTION  # Total depth in cm
depth_labels = {int(i / SAMPLE_RESOLUTION): f"{i / 100}" for i in range(0, int(MAX_DEPTH), Y_LABEL_DISTANCE)}


# Sonar Display Parameters
DISPLAY_GAIN = 1.3
DESPECKLE_WINDOW = 3
DESPECKLE_THRESHOLD = 10
SMOOTH_ALPHA = 0.25
TVG_STRENGTH = 1.2

# --- Helper Functions ---
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

# --- Thread Classes ---
class SerialReader(QThread):
    data_received = pyqtSignal(np.ndarray, float, float, float)
    def __init__(self, port, baud_rate):
        super().__init__()
        self.port = port
        self.baud_rate = baud_rate
        self.running = True
        self.send_queue = queue.Queue()
    def send_raw_command(self, cmd_bytes): self.send_queue.put(cmd_bytes)
    def stop(self):
        self.running = False
        self.quit()
        self.wait()
    def run(self):
        try:
            with serial.Serial(self.port, self.baud_rate, timeout=0.1, write_timeout=1) as ser:
                while self.running:
                    try:
                        while not self.send_queue.empty():
                            cmd = self.send_queue.get_nowait()
                            ser.write(cmd)
                            ser.flush()
                    except queue.Empty: pass
                    result = read_packet(ser)
                    if result: self.data_received.emit(*result)
        except Exception as e: print(f"❌ Serial Error: {e}")

class UDPReader(QThread):
    data_received = pyqtSignal(np.ndarray, float, float, float)
    def __init__(self, port: int):
        super().__init__()
        self.port = port
        self.running = True
        self._sock = None
    def run(self):
        try:
            self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self._sock.settimeout(1.0)
            self._sock.bind(("", self.port))
            while self.running:
                try:
                    datagram, _ = self._sock.recvfrom(PACKET_SIZE + 100)
                    if len(datagram) >= PACKET_SIZE and datagram[0] == 0xAA:
                        payload = datagram[1:1+6+NUM_SAMPLES]
                        depth, freq, vdrv = struct.unpack("<HhH", payload[:6])
                        values = np.frombuffer(payload[6:6+NUM_SAMPLES], dtype=np.uint8)
                        self.data_received.emit(values, depth, freq, float(vdrv))
                except socket.timeout: continue
        finally:
            if self._sock: self._sock.close()
    def stop(self):
        self.running = False
        self.wait()

# --- Settings Dialog ---
class SettingsDialog(QDialog):
    def __init__(self, parent=None, current_gradient="cyclic", current_speed=343, nmea_enabled=False, nmea_port=10110, nmea_address="127.0.0.1"):
        super().__init__(parent)
        self.main_app = parent
        self.setWindowTitle("Chart Settings")
        self.setFixedSize(360, 580) # 再增加一點高度

        main_layout = QVBoxLayout(self)
        
        # --- Appearance ---
        appearance_group = QWidget()
        appearance_layout = QFormLayout(appearance_group)
        # ... (原本的 Color Map 和 Sound Speed 保持不變) ...
        self.gradient_dropdown = QComboBox()
        self.gradient_dropdown.addItems(["viridis", "plasma", "inferno", "magma", "thermal", "flame", "yellowy", "bipolar", "spectrum", "cyclic", "greyclip", "grey"])
        self.gradient_dropdown.setCurrentText(current_gradient)
        self.speed_dropdown = QComboBox()
        self.speed_dropdown.addItems(["343 m/s (Air)", "1440 m/s (Water)"])
        self.speed_dropdown.setCurrentIndex(1 if current_speed == 1440 else 0)
        appearance_layout.addRow("Color Map:", self.gradient_dropdown)
        appearance_layout.addRow("Sound Speed:", self.speed_dropdown)
        main_layout.addWidget(QLabel("<b>Appearance</b>"))
        main_layout.addWidget(appearance_group)

        # --- Display Options (更新部分) ---
        main_layout.addWidget(QLabel("<b>Display Options</b>"))
        display_group = QWidget()
        display_layout = QFormLayout(display_group)

        # 1. Overlay 設定
        self.large_depth_checkbox = QCheckBox("Enable Depth Overlay")
        self.large_depth_checkbox.setChecked(self.main_app.large_depth_visible)
        
        self.overlay_mode_combo = QComboBox()
        self.overlay_mode_combo.addItems(["Auto (Threshold)", "Override (Max)"])
        self.overlay_mode_combo.setCurrentIndex(0 if self.main_app.depth_overlay_mode == "Auto" else 1)
        
        # 2. 紅線設定
        self.show_line_checkbox = QCheckBox("Show Red Depth Line")
        self.show_line_checkbox.setChecked(self.main_app.show_depth_line)

        self.line_mode_combo = QComboBox() # 新增：紅線模式選單
        self.line_mode_combo.addItems(["Follow Auto", "Follow Override"])
        self.line_mode_combo.setCurrentIndex(0 if self.main_app.depth_line_mode == "Auto" else 1)

        display_layout.addRow(self.large_depth_checkbox)
        display_layout.addRow("Overlay Source:", self.overlay_mode_combo)
        display_layout.addRow(QFrame()) # 分隔線感
        display_layout.addRow(self.show_line_checkbox)
        display_layout.addRow("Line Follow Mode:", self.line_mode_combo)
        
        # 連動控制
        self.large_depth_checkbox.toggled.connect(self.overlay_mode_combo.setEnabled)
        self.show_line_checkbox.toggled.connect(self.line_mode_combo.setEnabled)
        
        self.overlay_mode_combo.setEnabled(self.main_app.large_depth_visible)
        self.line_mode_combo.setEnabled(self.main_app.show_depth_line)

        main_layout.addWidget(display_group)

        # --- NMEA Section ---
        # ... (原本的 NMEA 區塊保持不變) ...
        nmea_group = QWidget()
        nmea_layout = QFormLayout(nmea_group)
        self.nmea_enable_checkbox = QCheckBox("Enable NMEA TCP Output")
        self.nmea_enable_checkbox.setChecked(nmea_enabled)
        self.port_input = QLineEdit(str(nmea_port))
        self.port_input.setEnabled(nmea_enabled)
        self.nmea_enable_checkbox.toggled.connect(self.port_input.setEnabled)
        nmea_layout.addRow("Enable:", self.nmea_enable_checkbox)
        nmea_layout.addRow("TCP Port:", self.port_input)
        nmea_layout.addRow("Address:", QLabel(nmea_address))
        main_layout.addWidget(QLabel("<b>NMEA Output</b>"))
        main_layout.addWidget(nmea_group)

        # --- Buttons ---
        button_row = QHBoxLayout()
        apply_btn = QPushButton("Apply")
        apply_btn.clicked.connect(self.apply_settings)
        cancel_btn = QPushButton("Cancel")
        cancel_btn.clicked.connect(self.close)
        button_row.addStretch()
        button_row.addWidget(apply_btn)
        button_row.addWidget(cancel_btn)
        main_layout.addStretch()
        main_layout.addLayout(button_row)

        self.setStyleSheet("QWidget { background-color: #2b2b2b; color: white; } QComboBox, QLineEdit { background-color: #3c3c3c; color: white; padding: 4px; } QPushButton { background-color: #444; border: 1px solid #666; padding: 6px 14px; border-radius: 4px; }")

    def apply_settings(self):
        # ... (讀取其他設定) ...
        if self.main_app:
            self.main_app.set_gradient(self.gradient_dropdown.currentText())
            self.main_app.set_sound_speed(343 if self.speed_dropdown.currentIndex() == 0 else 1440)
            
            # 套用 Overlay 設定
            self.main_app.large_depth_visible = self.large_depth_checkbox.isChecked()
            self.main_app.depth_overlay.setVisible(self.main_app.large_depth_visible)
            self.main_app.depth_overlay_mode = "Auto" if self.overlay_mode_combo.currentIndex() == 0 else "Override"
            
            # 套用紅線設定
            self.main_app.show_depth_line = self.show_line_checkbox.isChecked()
            self.main_app.depth_line_mode = "Auto" if self.line_mode_combo.currentIndex() == 0 else "Override"
            
            if not self.main_app.show_depth_line:
                self.main_app.depth_line.hide()

            self.main_app.configure_nmea_output(enabled=self.nmea_enable_checkbox.isChecked(), port=int(self.port_input.text()))
        self.close()

# --- Main App ---
class WaterfallApp(QMainWindow):
    def __init__(self):
        super().__init__()
        self.serial_thread = None
        self.udp_thread = None
        self.nmea_output_enabled = False
        self.nmea_client_socket = None
        self.nmea_server_socket = None
        self.nmea_port = 10110
        self.large_depth_visible = True
        self.current_gradient = 'cyclic' 
        self.current_speed = SPEED_OF_SOUND
        self.large_depth_visible = True
        self.depth_overlay_mode = "Auto"  # 可選 "Auto" (depth_m) 或 "Override" (ovrride_depth_m)
        self.show_depth_line = True      # 控制紅色水平線開關
        self.depth_line_mode = "Auto" # 可選 "Auto" (depth_m) 或 "Override" (ovrride_depth_m)
        

        self.setWindowTitle("Open Echo Interface")
        self.setGeometry(0, 0, 850, 850)
        self.data = np.zeros((MAX_ROWS, NUM_SAMPLES))

        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)
        main_layout.setContentsMargins(15, 10, 15, 10)
        main_layout.setSpacing(10)

        # [ Row 1 ] UDP (Left) & Serial Port (Right) 修正版
        row1_layout = QHBoxLayout()
        # --- 設定統一的字體大小 (您可以修改這裡的數字) ---
        target_font_size = 10
        row1_font = QFont("Arial", target_font_size)
        # 如果想要粗體，可以加上這行: row1_font.setBold(True)
        # ----------------------------------------------
        # UDP 部分
        label_udp = QLabel("UDP Port:")
        label_udp.setFont(row1_font) # 套用字體
        row1_layout.addWidget(label_udp)
        self.udp_port_input = QLineEdit("5005")
        self.udp_port_input.setFixedWidth(80) # 稍微加寬以容納大字體
        self.udp_port_input.setFont(row1_font) # 套用字體
        row1_layout.addWidget(self.udp_port_input)
        self.udp_connect_button = QPushButton("Connect UDP")
        self.udp_connect_button.setFont(row1_font) # 套用字體
        self.udp_connect_button.clicked.connect(self.toggle_udp_connection)
        row1_layout.addWidget(self.udp_connect_button)
        row1_layout.addStretch() # 彈簧推向右邊
        # Serial 部分
        label_serial = QLabel("Port:")
        label_serial.setFont(row1_font) # 套用字體
        row1_layout.addWidget(label_serial)
        self.serial_dropdown = QComboBox()
        self.serial_dropdown.addItems(get_serial_ports())
        self.serial_dropdown.setFixedWidth(150) # 稍微加寬以容納大字體
        self.serial_dropdown.setFont(row1_font) # 套用字體
        row1_layout.addWidget(self.serial_dropdown)
        self.connect_button = QPushButton("Connect")
        self.connect_button.setFont(row1_font) # 套用字體
        self.connect_button.clicked.connect(self.toggle_serial_connection)
        row1_layout.addWidget(self.connect_button)
        main_layout.addLayout(row1_layout)

        # [ Row 2 ] Echogram with Floating Depth Overlay
        self.waterfall = pg.PlotWidget()
        #self.waterfall.setContentsMargins(0, 0, 0, 20) # 左, 上, 右, 下
        self.imageitem = pg.ImageItem(axisOrder="row-major")
        self.waterfall.addItem(self.imageitem)
        self.waterfall.invertY(True)
        # 深度大字 (保持您之前的設定)
        self.depth_overlay = pg.TextItem(text="--- m", color=(255, 255, 255), anchor=(0, 1))
        f_overlay = QFont("Arial", 60, QFont.Bold)
        self.depth_overlay.setFont(f_overlay)
        self.waterfall.addItem(self.depth_overlay)
        # 設定 X 軸刻度字體 (可跟隨 Y 軸或獨立設定)
        x_axis = self.waterfall.getAxis("bottom")
        # 強制設定 X 軸的高度，確保大字體有足夠空間顯示
        x_axis.setHeight(40) 
        x_tick_font = QFont("Arial", 10) 
        self.waterfall.getAxis("bottom").setTickFont(x_tick_font)
        # --- 動態計算 Y 軸座標字體大小 ---
        # 取得刻度總數
        num_ticks = len(depth_labels)
        # 定義基礎字體大小邏輯：
        # 假設刻度少於 5 個時使用 18 號字，多於 20 個時縮小到 10 號字
        dynamic_tick_size = int(18 - (num_ticks - 5) * (8 / 15))
        tick_font = QFont("Arial", dynamic_tick_size)
        self.waterfall.getAxis("left").setTickFont(tick_font)
        self.waterfall.getAxis("right").setTickFont(tick_font)
        # -----------------------------
        inverted_depth_labels = list(depth_labels.items())[::-1]
        self.waterfall.getAxis("left").setTicks([inverted_depth_labels])
        self.waterfall.getAxis("right").setTicks([inverted_depth_labels])
        self.waterfall.getAxis("right").setStyle(showValues=True)
        # 繪製紅線與參考線
        self.depth_line = pg.InfiniteLine(angle=0, pen=pg.mkPen("r", width=2))
        self.waterfall.addItem(self.depth_line)
        for i in range(0, int(MAX_DEPTH), Y_LABEL_DISTANCE):
            row_index = int(i / SAMPLE_RESOLUTION)
            hline = pg.InfiniteLine(pos=row_index, angle=0, pen=pg.mkPen(color="w", style=Qt.DotLine))
            self.waterfall.addItem(hline)
        main_layout.addWidget(self.waterfall, stretch=1)

        # [ Row 4 ] Detailed Info Labels
        row4_layout = QHBoxLayout()
        row4_layout.setSpacing(30)
        self.depth_label = QLabel("Depth: --- cm")
        self.freq_label = QLabel("Drive Frequency: --- kHz")
        self.override_idx_label = QLabel("Override Idx: ---")
        info_style = "font-size: 20px; color: #ffffff; font-weight: bold;"
        for lbl in [self.depth_label, self.freq_label, self.override_idx_label]:
            lbl.setStyleSheet(info_style)
            row4_layout.addWidget(lbl)
        row4_layout.addStretch()
        main_layout.addLayout(row4_layout)

        # [ Row 5 ] Commands & Buttons
        row5_layout = QHBoxLayout()
        row5_layout.setSpacing(10)
        row5_font = QFont("Arial", target_font_size) 
        #row5_font.setBold(True) # 如果字很大，粗體會更清晰
        # 指令輸入框
        self.hex_input = QLineEdit()
        self.hex_input.setPlaceholderText("Addr, Data")
        self.hex_input.setFixedWidth(250) # 增加寬度以容納大字體
        self.hex_input.setFont(row5_font)
        row5_layout.addWidget(self.hex_input)
        # Send 按鈕
        self.send_button = QPushButton("Send")
        self.send_button.setFont(row5_font)
        self.send_button.clicked.connect(self.send_hex_value)
        row5_layout.addWidget(self.send_button)
        row5_layout.addStretch()
        # Settings 按鈕
        self.settings_button = QPushButton("Settings")
        self.settings_button.setFont(row5_font)
        self.settings_button.clicked.connect(self.open_settings)
        row5_layout.addWidget(self.settings_button)
        # Quit 按鈕
        self.quit_button = QPushButton("Quit")
        self.quit_button.setFont(row5_font)
        self.quit_button.clicked.connect(self.close)
        row5_layout.addWidget(self.quit_button)
        main_layout.addLayout(row5_layout)
        # Colorbar 與數據範圍設定保持不變
        self.colorbar = pg.HistogramLUTWidget()
        self.colorbar.setImageItem(self.imageitem)
        self.colorbar.item.gradient.loadPreset("cyclic") 
        self.imageitem.setLevels(DEFAULT_LEVELS)

    def waterfall_plot_callback(self, spectrogram, depth_index, drive_frequency, override_idx):
        """
        處理聲納數據回傳的 callback 函式。
        包含：瀑布圖更新、浮動文字顯示邏輯、以及水平紅線追蹤邏輯。
        """
        # 1. 影像處理與瀑布圖滾動更新
        filtered_line = sonar_display_pipeline(spectrogram)
        self.data = np.roll(self.data, -1, axis=0)
        self.data[-1, :] = filtered_line
        self.imageitem.setImage(self.data.T, autoLevels=False)
        self.imageitem.setLevels((10, 220)) # 設定顯示色階範圍
        
        # 2. 深度數據計算 (單位：公尺)
        depth_m = (depth_index * SAMPLE_RESOLUTION) / 100.0
        ovrride_depth_m = (override_idx * SAMPLE_RESOLUTION) / 100.0
        
        # 3. 計算誤差 (判斷 Auto 偵測是否與 Override 吻合)
        index_diff = abs(depth_index - override_idx)
        is_valid = index_diff <= INDEX_TOLERANCE
        
        # 4. --- 處理水平紅線 (Depth Line) 邏輯 ---
        if self.show_depth_line:
            # 決定紅線要追蹤哪一個數值
            if self.depth_line_mode == "Auto":
                target_idx = depth_index
            else:
                target_idx = override_idx
            
            self.depth_line.setPos(target_idx) # 設定線條在 Y 軸的位置
            self.depth_line.show()
            
            # 視覺反饋：如果追蹤 Auto 且數據不穩，將線條變淡變細
            if self.depth_line_mode == "Auto" and not is_valid:
                self.depth_line.setPen(pg.mkPen((255, 0, 0, 120), width=2)) # 半透明紅
            else:
                self.depth_line.setPen(pg.mkPen((255, 0, 0), width=3))      # 實心亮紅
        else:
            self.depth_line.hide() # 在設定中關閉紅線時隱藏

        # 5. --- 處理浮動深度文字 (Depth Overlay) 邏輯 ---
        if self.large_depth_visible:
            # 決定 Overlay 要顯示哪一個數值
            display_val = depth_m if self.depth_overlay_mode == "Auto" else ovrride_depth_m
            
            # 更新位置 (固定在左下角，座標 10, NUM_SAMPLES)
            self.depth_overlay.setPos(10, NUM_SAMPLES)
            
            if is_valid:
                # 數據吻合：顯示白色
                self.depth_overlay.setColor((255, 255, 255))
                self.depth_overlay.setText(f"{display_val:.1f} m")
            else:
                # 數據異常：顯示淡紅色警告
                self.depth_overlay.setColor((255, 100, 100))
                if self.depth_overlay_mode == "Auto":
                    self.depth_overlay.setText("0.0 m") # Auto 模式在警告時顯示 0.0
                else:
                    self.depth_overlay.setText(f"{display_val:.1f} m") # Override 模式則維持原值
        
        # 6. 更新介面底部的狀態標籤 (QLabel)
        self.freq_label.setText(f"Drive Frequency: {drive_frequency:.1f} kHz")
        self.depth_label.setText(f"Depth: {depth_m*100:.1f} cm ({depth_index:.0f})")
        self.override_idx_label.setText(f"Override Depth: {ovrride_depth_m*100:.1f} cm ({override_idx:.0f})")

        # 7. NMEA 輸出 (如果功能啟動且 Socket 已連線)
        if self.nmea_output_enabled and hasattr(self, 'nmea_client_socket') and self.nmea_client_socket:
            try:
                # 輸出當前回波深度至 NMEA 客戶端
                sentence = self.generate_dbt_sentence(depth_m * 100)
                self.nmea_client_socket.send(sentence.encode())
            except Exception:
                # 若傳送失敗，關閉 NMEA 輸出狀態
                self.nmea_output_enabled = False

    def toggle_serial_connection(self):
        if self.serial_thread and self.serial_thread.isRunning():
            self.serial_thread.stop()
            self.connect_button.setText("Connect")
        else:
            self.serial_thread = SerialReader(self.serial_dropdown.currentText(), BAUD_RATE)
            self.serial_thread.data_received.connect(self.waterfall_plot_callback)
            self.serial_thread.start()
            self.connect_button.setText("Disconnect")

    def toggle_udp_connection(self):
        if self.udp_thread and self.udp_thread.isRunning():
            self.udp_thread.stop()
            self.udp_connect_button.setText("Connect UDP")
        else:
            try:
                self.udp_thread = UDPReader(int(self.udp_port_input.text()))
                self.udp_thread.data_received.connect(self.waterfall_plot_callback)
                self.udp_thread.start()
                self.udp_connect_button.setText("Disconnect UDP")
            except: pass

    def send_hex_value(self):
        raw_text = self.hex_input.text().strip()
        if ',' in raw_text:
            try:
                addr, data = [int(x.strip(), 16) for x in raw_text.split(',')]
                if self.serial_thread and self.serial_thread.isRunning():
                    self.serial_thread.send_raw_command(struct.pack('BBB', ord('W'), addr, data))
            except: print("❌ Hex input error")

    def set_gradient(self, gradient_name):
        self.current_gradient = gradient_name
        self.colorbar.item.gradient.loadPreset(gradient_name)

    def set_sound_speed(self, speed):
        global SPEED_OF_SOUND, SAMPLE_RESOLUTION, MAX_DEPTH, depth_labels
        SPEED_OF_SOUND = self.current_speed = speed
        SAMPLE_RESOLUTION = (SPEED_OF_SOUND * SAMPLE_TIME * 100) / 2
        MAX_DEPTH = NUM_SAMPLES * SAMPLE_RESOLUTION
        # 1. 重新計算刻度標籤
        depth_labels = {int(i / SAMPLE_RESOLUTION): f"{i / 100}" for i in range(0, int(MAX_DEPTH), Y_LABEL_DISTANCE)}
        inverted = list(depth_labels.items())[::-1]
        # 2. 根據新的刻度數量計算字體大小
        num_ticks = len(depth_labels)
        # 線性計算：刻度越多字越小 (從 18 降到 10)
        dynamic_size = int(18 - (num_ticks - 5) * (8 / 15))
        tick_font = QFont("Arial", dynamic_size)
        # 3. 更新 Y 軸刻度內容與字體大小
        left_axis = self.waterfall.getAxis("left")
        right_axis = self.waterfall.getAxis("right")
        left_axis.setTickFont(tick_font)
        right_axis.setTickFont(tick_font)
        left_axis.setTicks([inverted])
        right_axis.setTicks([inverted])

    def configure_nmea_output(self, enabled, port):
        self.nmea_output_enabled, self.nmea_port = enabled, port

    def open_settings(self):
        try:
            dlg = SettingsDialog(self, self.current_gradient, self.current_speed, self.nmea_output_enabled, self.nmea_port, get_local_ip())
            dlg.setWindowModality(Qt.ApplicationModal)
            dlg.exec_()
        except Exception as e: print(f"❌ Settings error: {e}")

    def closeEvent(self, event):
        if self.serial_thread: self.serial_thread.stop()
        if self.udp_thread: self.udp_thread.stop()
        event.accept()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    qdarktheme.setup_theme("dark")
    window = WaterfallApp()
    window.show()
    sys.exit(app.exec())
