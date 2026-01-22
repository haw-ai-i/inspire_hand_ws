"""
Safe Hand Control UI for Inspire RH56DFTP Hands

A PyQt5 GUI for controlling both left and right Inspire dexterous hands
with safety guardrails:
- Slew rate limiting (smooth movements)
- Hardware speed/force limiting
- Emergency stop
- Full diagnostics (errors, status, force, temperature)

Usage:
    source /home/esports/Documents/unitree/inspire_hand_ws/hands-env/bin/activate
    python hand_control_ui.py

Author: Generated for Unitree H1 with Inspire Hands
"""

import sys
import time
import numpy as np
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QGridLayout, QLabel, QSlider, QPushButton, QGroupBox, QCheckBox,
    QSpinBox, QStatusBar, QComboBox, QProgressBar, QTabWidget
)
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QFont, QColor, QPalette
from pymodbus.client import ModbusTcpClient
import struct


# Hand configuration
HAND_CONFIG = {
    'left': {'ip': '192.168.123.210', 'port': 6000, 'device_id': 1},
    'right': {'ip': '192.168.123.211', 'port': 6000, 'device_id': 1},
}

# Joint names
JOINT_NAMES = ['Pinky', 'Ring', 'Middle', 'Index', 'Thumb Bend', 'Thumb Rot']

# Safety limits
POSITION_MIN = 0     # Hardware minimum
POSITION_MAX = 1000  # Hardware maximum
DEFAULT_SPEED = 500  # Hardware speed limit
DEFAULT_FORCE = 300  # Hardware force limit
MAX_SLEW_RATE = 25   # Max units per update cycle (lower = safer, 25 = 2sec full travel)
UPDATE_RATE_MS = 50  # 20 Hz update rate

# Status code translations (from inspire_hand_defaut.py)
STATUS_CODES = {
    0: "Releasing",
    1: "Grasping",
    2: "Position Reached",
    3: "Force Limit Reached",
    5: "Current Protection",
    6: "Stall Stop",
    7: "Fault Stop",
    255: "Error"
}

# Error bit descriptions
ERROR_BITS = {
    0: "Stall",
    1: "Overtemp",
    2: "Overcurrent",
    3: "Motor Fault",
    4: "Comm Fault"
}


class HandController:
    """Handles Modbus communication with one hand."""
    
    def __init__(self, name, ip, port, device_id):
        self.name = name
        self.ip = ip
        self.port = port
        self.device_id = device_id
        self.client = None
        self.connected = False
        
        # Control mode: 'angle' or 'position'
        self.control_mode = 'angle'
        
        # Current state
        self.current_positions = [0] * 6   # pos_act
        self.current_angles = [0] * 6      # angle_act
        self.current_forces = [0] * 6      # force_act
        self.current_currents = [0] * 6    # current
        self.current_errors = [0] * 6      # err
        self.current_status = [0] * 6      # status
        self.current_temps = [0] * 6       # temperature
        
        # Target/command positions
        self.target_values = [0] * 6       # Where sliders are set
        self.command_values = [0] * 6      # What we actually send (slew-limited)
        
    def connect(self):
        """Connect to the hand via Modbus TCP."""
        try:
            self.client = ModbusTcpClient(self.ip, port=self.port)
            if self.client.connect():
                self.connected = True
                # Clear any errors
                self.client.write_register(1004, 1, self.device_id)
                print(f"{self.name} hand connected at {self.ip}")
                
                # Read current state and initialize targets to match
                self.read_full_state()
                # Use angle values as default
                self.target_values = self.current_angles.copy()
                self.command_values = self.current_angles.copy()
                
                return True
        except Exception as e:
            print(f"Failed to connect to {self.name} hand: {e}")
        return False
    
    def disconnect(self):
        """Disconnect from the hand."""
        if self.client:
            self.client.close()
            self.connected = False
    
    def _read_registers(self, address, count, data_type='short'):
        """Read and parse registers."""
        try:
            response = self.client.read_holding_registers(address, count, self.device_id)
            if not response.isError():
                if data_type == 'short':
                    packed = struct.pack('>' + 'H' * count, *response.registers)
                    return list(struct.unpack('>' + 'h' * count, packed))
                elif data_type == 'byte':
                    result = []
                    for reg in response.registers:
                        result.append((reg >> 8) & 0xFF)
                        result.append(reg & 0xFF)
                    return result
        except Exception as e:
            print(f"Error reading registers at {address}: {e}")
        return None
            
    def read_full_state(self):
        """Read all state registers from the hand."""
        if not self.connected:
            return False
        try:
            # Position actual (register 1534)
            vals = self._read_registers(1534, 6, 'short')
            if vals:
                self.current_positions = vals
                
            # Angle actual (register 1546)
            vals = self._read_registers(1546, 6, 'short')
            if vals:
                self.current_angles = vals
                
            # Force actual (register 1582)
            vals = self._read_registers(1582, 6, 'short')
            if vals:
                self.current_forces = vals
                
            # Current (register 1594)
            vals = self._read_registers(1594, 6, 'short')
            if vals:
                self.current_currents = vals
                
            # Error codes (register 1606, 3 regs = 6 bytes)
            vals = self._read_registers(1606, 3, 'byte')
            if vals:
                self.current_errors = vals
                
            # Status (register 1612, 3 regs = 6 bytes)
            vals = self._read_registers(1612, 3, 'byte')
            if vals:
                self.current_status = vals
                
            # Temperature (register 1618, 3 regs = 6 bytes)
            vals = self._read_registers(1618, 3, 'byte')
            if vals:
                self.current_temps = vals
                
            return True
        except Exception as e:
            print(f"Error reading {self.name} hand state: {e}")
        return False
    
    def send_command(self, values, speed=DEFAULT_SPEED, force=DEFAULT_FORCE):
        """Send position/angle command to the hand."""
        if not self.connected:
            return False
        try:
            # Set speed limit
            self.client.write_registers(1522, [speed] * 6, self.device_id)
            # Set force limit
            self.client.write_registers(1498, [force] * 6, self.device_id)
            
            # Send based on control mode
            if self.control_mode == 'angle':
                self.client.write_registers(1486, values, self.device_id)
            else:  # position mode
                self.client.write_registers(1474, values, self.device_id)
            return True
        except Exception as e:
            print(f"Error sending command to {self.name} hand: {e}")
        return False
    
    def emergency_stop(self):
        """Open all fingers immediately."""
        if not self.connected:
            return
        try:
            # High speed, low force, open position
            self.client.write_registers(1522, [1000] * 6, self.device_id)
            self.client.write_registers(1498, [100] * 6, self.device_id)
            self.client.write_registers(1486, [0] * 6, self.device_id)
            self.target_values = [0] * 6
            self.command_values = [0] * 6
        except Exception as e:
            print(f"Emergency stop error on {self.name}: {e}")
    
    def get_error_string(self, error_val):
        """Convert error byte to readable string."""
        if error_val == 0:
            return "OK"
        errors = []
        for bit, desc in ERROR_BITS.items():
            if error_val & (1 << bit):
                errors.append(desc)
        return ", ".join(errors) if errors else "Unknown"
    
    def get_status_string(self, status_val):
        """Convert status byte to readable string."""
        return STATUS_CODES.get(status_val, f"Unknown({status_val})")


class HandPanel(QWidget):
    """UI panel for controlling one hand."""
    
    def __init__(self, name, controller):
        super().__init__()
        self.name = name
        self.controller = controller
        self.sliders = []
        self.value_labels = []
        self.force_bars = []
        self.status_labels = []
        self.error_labels = []
        self.temp_labels = []
        self.enabled = False
        
        self.init_ui()
        
    def init_ui(self):
        layout = QVBoxLayout()
        
        # Header with enable checkbox and control mode
        header = QHBoxLayout()
        self.enable_checkbox = QCheckBox(f"Enable {self.name.title()} Hand")
        self.enable_checkbox.stateChanged.connect(self.on_enable_changed)
        header.addWidget(self.enable_checkbox)
        
        header.addWidget(QLabel("Control:"))
        self.mode_combo = QComboBox()
        self.mode_combo.addItems(["Angle", "Position"])
        self.mode_combo.currentTextChanged.connect(self.on_mode_changed)
        header.addWidget(self.mode_combo)
        
        self.status_label = QLabel("Disconnected")
        self.status_label.setStyleSheet("color: red; font-weight: bold;")
        header.addWidget(self.status_label)
        header.addStretch()
        layout.addLayout(header)
        
        # Create tabs for Control and Diagnostics
        tabs = QTabWidget()
        
        # === Control Tab ===
        control_widget = QWidget()
        control_layout = QVBoxLayout()
        
        grid = QGridLayout()
        grid.addWidget(QLabel("Joint"), 0, 0)
        grid.addWidget(QLabel("Target"), 0, 1)
        grid.addWidget(QLabel("Value"), 0, 2)
        grid.addWidget(QLabel("Force"), 0, 3)
        
        for i, joint_name in enumerate(JOINT_NAMES):
            label = QLabel(joint_name)
            grid.addWidget(label, i + 1, 0)
            
            slider = QSlider(Qt.Horizontal)
            slider.setMinimum(POSITION_MIN)
            slider.setMaximum(POSITION_MAX)
            slider.setValue(0)
            slider.setEnabled(False)
            slider.valueChanged.connect(lambda v, idx=i: self.on_slider_changed(idx, v))
            self.sliders.append(slider)
            grid.addWidget(slider, i + 1, 1)
            
            value_label = QLabel("---")
            value_label.setMinimumWidth(50)
            self.value_labels.append(value_label)
            grid.addWidget(value_label, i + 1, 2)
            
            force_bar = QProgressBar()
            force_bar.setRange(0, 1000)
            force_bar.setValue(0)
            force_bar.setTextVisible(False)
            force_bar.setMaximumWidth(80)
            self.force_bars.append(force_bar)
            grid.addWidget(force_bar, i + 1, 3)
            
        control_layout.addLayout(grid)
        
        # Preset buttons
        presets = QHBoxLayout()
        for name, func in [("Open All", self.open_all), ("Close All", self.close_all),
                           ("Pinch", self.preset_pinch), ("Power Grip", self.preset_power)]:
            btn = QPushButton(name)
            btn.clicked.connect(func)
            presets.addWidget(btn)
        control_layout.addLayout(presets)
        
        control_widget.setLayout(control_layout)
        tabs.addTab(control_widget, "Control")
        
        # === Diagnostics Tab ===
        diag_widget = QWidget()
        diag_layout = QVBoxLayout()
        
        diag_grid = QGridLayout()
        diag_grid.addWidget(QLabel("Joint"), 0, 0)
        diag_grid.addWidget(QLabel("Status"), 0, 1)
        diag_grid.addWidget(QLabel("Error"), 0, 2)
        diag_grid.addWidget(QLabel("Temp °C"), 0, 3)
        diag_grid.addWidget(QLabel("Current"), 0, 4)
        
        for i, joint_name in enumerate(JOINT_NAMES):
            diag_grid.addWidget(QLabel(joint_name), i + 1, 0)
            
            status_lbl = QLabel("---")
            self.status_labels.append(status_lbl)
            diag_grid.addWidget(status_lbl, i + 1, 1)
            
            error_lbl = QLabel("---")
            self.error_labels.append(error_lbl)
            diag_grid.addWidget(error_lbl, i + 1, 2)
            
            temp_lbl = QLabel("---")
            self.temp_labels.append(temp_lbl)
            diag_grid.addWidget(temp_lbl, i + 1, 3)
            
            # Current is stored alongside other data
            
        diag_layout.addLayout(diag_grid)
        diag_widget.setLayout(diag_layout)
        tabs.addTab(diag_widget, "Diagnostics")
        
        layout.addWidget(tabs)
        self.setLayout(layout)
        
    def on_mode_changed(self, mode):
        self.controller.control_mode = mode.lower()
        
    def on_enable_changed(self, state):
        self.enabled = state == Qt.Checked
        if self.enabled:
            if self.controller.connect():
                self.status_label.setText("Connected")
                self.status_label.setStyleSheet("color: green; font-weight: bold;")
                
                # Initialize sliders to current values (prevents jumps!)
                for i, slider in enumerate(self.sliders):
                    slider.blockSignals(True)
                    slider.setValue(self.controller.current_angles[i])
                    slider.blockSignals(False)
                    slider.setEnabled(True)
            else:
                self.enable_checkbox.setChecked(False)
                self.enabled = False
        else:
            self.controller.disconnect()
            self.status_label.setText("Disconnected")
            self.status_label.setStyleSheet("color: red; font-weight: bold;")
            for slider in self.sliders:
                slider.setEnabled(False)
                
    def on_slider_changed(self, joint_idx, value):
        self.controller.target_values[joint_idx] = value
        
    def update_display(self):
        """Update all displays from current state."""
        ctrl = self.controller
        
        # Control tab
        for i in range(6):
            # Show angle or position depending on mode
            if ctrl.control_mode == 'angle':
                self.value_labels[i].setText(str(ctrl.current_angles[i]))
            else:
                self.value_labels[i].setText(str(ctrl.current_positions[i]))
            
            # Force bar
            force = max(0, min(1000, ctrl.current_forces[i]))
            self.force_bars[i].setValue(force)
            
        # Diagnostics tab
        for i in range(6):
            self.status_labels[i].setText(ctrl.get_status_string(ctrl.current_status[i]))
            
            error_str = ctrl.get_error_string(ctrl.current_errors[i])
            self.error_labels[i].setText(error_str)
            if error_str != "OK":
                self.error_labels[i].setStyleSheet("color: red; font-weight: bold;")
            else:
                self.error_labels[i].setStyleSheet("")
                
            self.temp_labels[i].setText(str(ctrl.current_temps[i]))
            
    def open_all(self):
        for slider in self.sliders:
            slider.setValue(POSITION_MIN)
            
    def close_all(self):
        for slider in self.sliders:
            slider.setValue(POSITION_MAX)
            
    def preset_pinch(self):
        values = [0, 0, 0, 700, 700, 500]
        for i, slider in enumerate(self.sliders):
            slider.setValue(values[i])
            
    def preset_power(self):
        values = [800, 800, 800, 800, 700, 300]
        for i, slider in enumerate(self.sliders):
            slider.setValue(values[i])


class HandControlUI(QMainWindow):
    """Main application window."""
    
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Inspire Hand Control - Safe Mode")
        self.setGeometry(100, 100, 1000, 600)
        
        self.left_controller = HandController(
            'left', HAND_CONFIG['left']['ip'],
            HAND_CONFIG['left']['port'], HAND_CONFIG['left']['device_id']
        )
        self.right_controller = HandController(
            'right', HAND_CONFIG['right']['ip'],
            HAND_CONFIG['right']['port'], HAND_CONFIG['right']['device_id']
        )
        
        self.init_ui()
        self.init_timer()
        
    def init_ui(self):
        central = QWidget()
        main_layout = QVBoxLayout()
        
        # Emergency stop button
        estop_btn = QPushButton("⚠️ EMERGENCY STOP ⚠️")
        estop_btn.setStyleSheet("""
            QPushButton {
                background-color: #ff4444; color: white;
                font-size: 18px; font-weight: bold;
                padding: 15px; border-radius: 10px;
            }
            QPushButton:hover { background-color: #ff0000; }
        """)
        estop_btn.clicked.connect(self.emergency_stop)
        main_layout.addWidget(estop_btn)
        
        # Hand panels
        hands_layout = QHBoxLayout()
        
        left_group = QGroupBox("Left Hand")
        left_layout = QVBoxLayout()
        self.left_panel = HandPanel('left', self.left_controller)
        left_layout.addWidget(self.left_panel)
        left_group.setLayout(left_layout)
        hands_layout.addWidget(left_group)
        
        right_group = QGroupBox("Right Hand")
        right_layout = QVBoxLayout()
        self.right_panel = HandPanel('right', self.right_controller)
        right_layout.addWidget(self.right_panel)
        right_group.setLayout(right_layout)
        hands_layout.addWidget(right_group)
        
        main_layout.addLayout(hands_layout)
        
        # Global controls
        global_layout = QHBoxLayout()
        
        global_layout.addWidget(QLabel("Speed:"))
        self.speed_spin = QSpinBox()
        self.speed_spin.setRange(100, 1000)
        self.speed_spin.setValue(DEFAULT_SPEED)
        global_layout.addWidget(self.speed_spin)
        
        global_layout.addWidget(QLabel("Force:"))
        self.force_spin = QSpinBox()
        self.force_spin.setRange(100, 1000)
        self.force_spin.setValue(DEFAULT_FORCE)
        global_layout.addWidget(self.force_spin)
        
        global_layout.addWidget(QLabel("Max Step:"))
        self.slew_spin = QSpinBox()
        self.slew_spin.setRange(5, 100)
        self.slew_spin.setValue(MAX_SLEW_RATE)
        self.slew_spin.setToolTip("Max change per cycle. 25 = 2sec full travel. Lower = safer.")
        global_layout.addWidget(self.slew_spin)
        
        # Velocity info
        self.velocity_label = QLabel()
        self.update_velocity_label()
        self.slew_spin.valueChanged.connect(self.update_velocity_label)
        global_layout.addWidget(self.velocity_label)
        
        global_layout.addStretch()
        main_layout.addLayout(global_layout)
        
        central.setLayout(main_layout)
        self.setCentralWidget(central)
        self.statusBar().showMessage("Ready - Enable hands to start")
        
    def update_velocity_label(self):
        slew = self.slew_spin.value()
        hz = 1000 / UPDATE_RATE_MS
        velocity_units = slew * hz  # units/sec
        travel_time = 1000 / velocity_units if velocity_units > 0 else float('inf')
        # Estimate: 0-1000 units ≈ 90° (1.57 rad) finger range
        velocity_rad = (velocity_units / 1000) * 1.57  # rad/sec
        self.velocity_label.setText(f"(~{velocity_rad:.2f} rad/s, {travel_time:.1f}s full travel)")
        
    def init_timer(self):
        self.timer = QTimer()
        self.timer.timeout.connect(self.control_loop)
        self.timer.start(UPDATE_RATE_MS)
        
    def control_loop(self):
        speed = self.speed_spin.value()
        force = self.force_spin.value()
        max_step = self.slew_spin.value()
        
        for panel, controller in [
            (self.left_panel, self.left_controller),
            (self.right_panel, self.right_controller)
        ]:
            if not panel.enabled:
                continue
                
            controller.read_full_state()
            panel.update_display()
            
            # Slew rate limiting
            for i in range(6):
                target = controller.target_values[i]
                current = controller.command_values[i]
                diff = target - current
                if abs(diff) > max_step:
                    diff = max_step if diff > 0 else -max_step
                controller.command_values[i] = current + diff
            
            controller.send_command(controller.command_values, speed, force)
            
    def emergency_stop(self):
        self.statusBar().showMessage("⚠️ EMERGENCY STOP ACTIVATED ⚠️")
        self.left_controller.emergency_stop()
        self.right_controller.emergency_stop()
        self.left_panel.open_all()
        self.right_panel.open_all()
        
    def closeEvent(self, event):
        self.timer.stop()
        self.left_controller.disconnect()
        self.right_controller.disconnect()
        event.accept()


def main():
    app = QApplication(sys.argv)
    app.setStyle('Fusion')
    window = HandControlUI()
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
