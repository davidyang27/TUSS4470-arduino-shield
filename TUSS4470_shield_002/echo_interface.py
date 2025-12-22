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
        self.setFixedSize(360, 450)

        main_layout = QVBoxLayout(self)
        
        # Appearance
        appearance_group = QWidget()
        appearance_layout = QFormLayout(appearance_group)
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

        # Display Options
        self.large_depth_checkbox = QCheckBox("Show Floating Depth Overlay")
        self.large_depth_checkbox.setChecked(getattr(parent, "large_depth_visible", True))
        main_layout.addWidget(self.large_depth_checkbox)

        # NMEA Section
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

        # Buttons
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
        gradient = self.gradient_dropdown.currentText()
        speed = 343 if self.speed_dropdown.currentIndex() == 0 else 1440
        nmea_enabled = self.nmea_enable_checkbox.isChecked()
        port = int(self.port_input.text()) if self.port_input.text().isdigit() else 10110
        
        if self.main_app:
            self.main_app.set_gradient(gradient)
            self.main_app.set_sound_speed(speed)
            self.main_app.large_depth_visible = self.large_depth_checkbox.isChecked()
            self.main_app.depth_overlay.setVisible(self.main_app.large_depth_visible)
            self.main_app.configure_nmea_output(enabled=nmea_enabled, port=port)
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

        self.setWindowTitle("Open Echo Interface")
        self.setGeometry(0, 0, 600, 850)
        self.data = np.zeros((MAX_ROWS, NUM_SAMPLES))

        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)
        main_layout.setContentsMargins(15, 10, 15, 10)
        main_layout.setSpacing(10)

        # [ Row 1 ] UDP (Left) & Serial Port (Right)
        row1_layout = QHBoxLayout()
        row1_layout.addWidget(QLabel("UDP Port:"))
        self.udp_port_input = QLineEdit("5005")
        self.udp_port_input.setFixedWidth(60)
        row1_layout.addWidget(self.udp_port_input)
        self.udp_connect_button = QPushButton("Connect UDP")
        self.udp_connect_button.clicked.connect(self.toggle_udp_connection)
        row1_layout.addWidget(self.udp_connect_button)
        row1_layout.addStretch()
        row1_layout.addWidget(QLabel("Port:"))
        self.serial_dropdown = QComboBox()
        self.serial_dropdown.addItems(get_serial_ports())
        self.serial_dropdown.setFixedWidth(100)
        row1_layout.addWidget(self.serial_dropdown)
        self.connect_button = QPushButton("Connect")
        self.connect_button.clicked.connect(self.toggle_serial_connection)
        row1_layout.addWidget(self.connect_button)
        main_layout.addLayout(row1_layout)

        # [ Row 2 ] Echogram with Floating Depth Overlay
        self.waterfall = pg.PlotWidget()
        self.imageitem = pg.ImageItem(axisOrder="row-major")
        self.waterfall.addItem(self.imageitem)
        self.waterfall.invertY(True)
        self.depth_overlay = pg.TextItem(text="--- m", color=(255, 255, 255), anchor=(0, 1))
        font = QFont("Arial", 60, QFont.Bold)
        self.depth_overlay.setFont(font)
        self.waterfall.addItem(self.depth_overlay)
        
        inverted_depth_labels = list(depth_labels.items())[::-1]
        self.waterfall.getAxis("left").setTicks([inverted_depth_labels])
        self.waterfall.getAxis("right").setTicks([inverted_depth_labels])
        self.waterfall.getAxis("right").setStyle(showValues=True)
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
        self.drive_voltage_label = QLabel("Override Idx: ---")
        info_style = "font-size: 14px; color: #ffffff; font-weight: bold;"
        for lbl in [self.depth_label, self.freq_label, self.drive_voltage_label]:
            lbl.setStyleSheet(info_style)
            row4_layout.addWidget(lbl)
        row4_layout.addStretch()
        main_layout.addLayout(row4_layout)

        # [ Row 5 ] Commands & Buttons
        row5_layout = QHBoxLayout()
        self.hex_input = QLineEdit()
        self.hex_input.setPlaceholderText("Addr, Data")
        self.hex_input.setFixedWidth(200)
        row5_layout.addWidget(self.hex_input)
        self.send_button = QPushButton("Send")
        self.send_button.clicked.connect(self.send_hex_value)
        row5_layout.addWidget(self.send_button)
        row5_layout.addStretch()
        self.settings_button = QPushButton("Settings")
        self.settings_button.clicked.connect(self.open_settings)
        row5_layout.addWidget(self.settings_button)
        self.quit_button = QPushButton("Quit")
        self.quit_button.clicked.connect(self.close)
        row5_layout.addWidget(self.quit_button)
        main_layout.addLayout(row5_layout)

        self.colorbar = pg.HistogramLUTWidget()
        self.colorbar.setImageItem(self.imageitem)
        self.colorbar.item.gradient.loadPreset("cyclic") 
        self.imageitem.setLevels(DEFAULT_LEVELS)

    def waterfall_plot_callback(self, spectrogram, depth_index, drive_frequency, drive_voltage):
        filtered_line = sonar_display_pipeline(spectrogram)
        self.data = np.roll(self.data, -1, axis=0)
        self.data[-1, :] = filtered_line
        self.imageitem.setImage(self.data.T, autoLevels=False)
        self.imageitem.setLevels((10, 220))
        depth_m = (depth_index * SAMPLE_RESOLUTION) / 100.0
        self.depth_overlay.setText(f"{depth_m:.1f} m")
        self.depth_overlay.setPos(10, NUM_SAMPLES)
        self.depth_label.setText(f"Depth: {depth_m*100:.1f} cm | Index: {depth_index:.0f}")
        self.freq_label.setText(f"Drive Frequency: {drive_frequency:.1f} kHz")
        self.drive_voltage_label.setText(f"Override Idx: {drive_voltage:.0f}")
        index_diff = abs(depth_index - drive_voltage)
        if index_diff <= INDEX_TOLERANCE:
            self.depth_line.setPos(depth_index)
            self.depth_line.show()
            self.depth_line.setPen(pg.mkPen((255, 0, 0), width=3))
            self.depth_overlay.setColor((255, 255, 255))
        else:
            self.depth_line.setPen(pg.mkPen((255, 0, 0, 120), width=2))
            self.depth_overlay.setColor((255, 100, 100))

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
        depth_labels = {int(i / SAMPLE_RESOLUTION): f"{i / 100}" for i in range(0, int(MAX_DEPTH), Y_LABEL_DISTANCE)}
        inverted = list(depth_labels.items())[::-1]
        self.waterfall.getAxis("left").setTicks([inverted])
        self.waterfall.getAxis("right").setTicks([inverted])

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
