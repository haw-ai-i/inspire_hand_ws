"""
DDS Hand Control UI (Inspire)

Controls Inspire hands over DDS topics and plots live motor current:
  - Publish: rt/inspire_hand/ctrl/l, rt/inspire_hand/ctrl/r
  - Subscribe: rt/inspire_hand/state/l, rt/inspire_hand/state/r

Usage:
  source /home/esports/Documents/teleop_setup/inspire_hand_ws/hands-env/bin/activate
  python inspire_hand_sdk/example/hand_control_ui_dds.py --network enp39s0
"""

from __future__ import annotations

import argparse
from collections import deque
import os
import threading
from typing import Optional

# Reduce Qt/GL driver-related native crashes on some systems.
os.environ.setdefault("QT_OPENGL", "software")
os.environ.setdefault("LIBGL_ALWAYS_SOFTWARE", "1")
os.environ.setdefault("QT_XCB_GL_INTEGRATION", "none")

import numpy as np
import pyqtgraph as pg
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtWidgets import (
    QApplication,
    QComboBox,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QPushButton,
    QSlider,
    QVBoxLayout,
    QWidget,
)

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher, ChannelSubscriber
from inspire_sdkpy import inspire_dds, inspire_hand_defaut


JOINT_NAMES = ["Pinky", "Ring", "Middle", "Index", "Thumb Bend", "Thumb Rot"]


class HandDDSIO:
    def __init__(self, side: str):
        assert side in ("l", "r")
        self.side = side
        self._state_lock = threading.Lock()
        self._state: Optional[inspire_dds.inspire_hand_state] = None

        self._pub = ChannelPublisher(f"rt/inspire_hand/ctrl/{self.side}", inspire_dds.inspire_hand_ctrl)
        self._pub.Init()

        self._sub = ChannelSubscriber(
            f"rt/inspire_hand/state/{self.side}",
            inspire_dds.inspire_hand_state,
        )
        self._sub.Init(self._on_state, 10)

    def _on_state(self, msg: inspire_dds.inspire_hand_state):
        with self._state_lock:
            self._state = msg

    def read_state(self) -> Optional[inspire_dds.inspire_hand_state]:
        with self._state_lock:
            return self._state

    def send_grasp(self, value: int, mode: str = "position"):
        value = int(np.clip(value, 0, 1000))
        cmd = inspire_hand_defaut.get_inspire_hand_ctrl()
        cmd.pos_set = [value] * 6
        cmd.angle_set = [value] * 6
        cmd.force_set = [300] * 6
        cmd.speed_set = [500] * 6
        # 0b0001 angle, 0b0010 position
        cmd.mode = 0b0010 if mode == "position" else 0b0001
        self._pub.Write(cmd)


class HandPlotCanvas(QWidget):
    def __init__(self, side_label: str, history_len: int, dt_s: float):
        super().__init__()
        self.history_len = history_len
        self.dt_s = dt_s
        self.x = np.linspace(-history_len * dt_s, 0.0, history_len)

        layout = QVBoxLayout(self)
        self.plot_curr = pg.PlotWidget(title=f"{side_label} Hand Current")
        self.plot_force = pg.PlotWidget(title=f"{side_label} Hand Force")
        self.plot_temp = pg.PlotWidget(title=f"{side_label} Hand Temperature")

        for plot in (self.plot_curr, self.plot_force, self.plot_temp):
            plot.showGrid(x=True, y=True, alpha=0.3)
            layout.addWidget(plot)

        self.plot_curr.setLabel("left", "Current")
        self.plot_force.setLabel("left", "Force")
        self.plot_temp.setLabel("left", "Temperature")
        self.plot_temp.setLabel("bottom", "Time", "s")

        colors = ["#d32f2f", "#f57c00", "#fbc02d", "#388e3c", "#1976d2", "#7b1fa2"]
        self.lines_curr = []
        self.lines_force = []
        self.lines_temp = []
        for i, name in enumerate(JOINT_NAMES):
            pen = pg.mkPen(colors[i % len(colors)], width=2)
            self.lines_curr.append(self.plot_curr.plot(self.x, np.zeros(history_len), pen=pen, name=name))
            self.lines_force.append(self.plot_force.plot(self.x, np.zeros(history_len), pen=pen, name=name))
            self.lines_temp.append(self.plot_temp.plot(self.x, np.zeros(history_len), pen=pen, name=name))

        self.plot_curr.addLegend()
        self.plot_force.addLegend()
        self.plot_temp.addLegend()

    def update_plot(
        self,
        curr_hist: list[deque],
        force_hist: list[deque],
        temp_hist: list[deque],
    ):
        for i in range(6):
            self.lines_curr[i].setData(self.x, np.array(curr_hist[i], dtype=float))
            self.lines_force[i].setData(self.x, np.array(force_hist[i], dtype=float))
            self.lines_temp[i].setData(self.x, np.array(temp_hist[i], dtype=float))


class MainWindow(QMainWindow):
    def __init__(self, dt_ms: int = 50, no_plot: bool = False):
        super().__init__()
        self.setWindowTitle("Inspire DDS Hand Control + Current Monitor")
        self.resize(1200, 800)

        self.dt_ms = dt_ms
        self.no_plot = no_plot
        self.dt_s = dt_ms / 1000.0
        self.history_len = 300
        self.tick_count = 0

        self.hand_l = HandDDSIO("l")
        self.hand_r = HandDDSIO("r")

        self.current_left_hist = [deque([0.0] * self.history_len, maxlen=self.history_len) for _ in range(6)]
        self.current_right_hist = [deque([0.0] * self.history_len, maxlen=self.history_len) for _ in range(6)]
        self.force_left_hist = [deque([0.0] * self.history_len, maxlen=self.history_len) for _ in range(6)]
        self.force_right_hist = [deque([0.0] * self.history_len, maxlen=self.history_len) for _ in range(6)]
        self.temp_left_hist = [deque([0.0] * self.history_len, maxlen=self.history_len) for _ in range(6)]
        self.temp_right_hist = [deque([0.0] * self.history_len, maxlen=self.history_len) for _ in range(6)]

        self.left_labels: list[QLabel] = []
        self.right_labels: list[QLabel] = []

        self._build_ui()

        self.timer = QTimer(self)
        self.timer.timeout.connect(self._tick)
        self.timer.start(self.dt_ms)

    def _build_ui(self):
        root = QWidget()
        main = QHBoxLayout(root)
        left_col = QVBoxLayout()
        right_col = QVBoxLayout()

        left_ctrl = QGroupBox("Left Hand Control")
        left_ctrl_layout = QGridLayout(left_ctrl)
        left_ctrl_layout.addWidget(QLabel("Mode"), 0, 0)
        self.left_mode_combo = QComboBox()
        self.left_mode_combo.addItems(["position", "angle"])
        left_ctrl_layout.addWidget(self.left_mode_combo, 0, 1)
        left_ctrl_layout.addWidget(QLabel("Grasp Slider"), 1, 0)
        self.left_slider = QSlider(Qt.Horizontal)
        self.left_slider.setRange(0, 1000)
        self.left_slider.setValue(0)
        left_ctrl_layout.addWidget(self.left_slider, 1, 1, 1, 2)
        self.left_slider_label = QLabel("0")
        left_ctrl_layout.addWidget(self.left_slider_label, 1, 3)
        self.left_slider.valueChanged.connect(lambda v: self.left_slider_label.setText(str(v)))
        btn_l_open = QPushButton("Open (0)")
        btn_l_half = QPushButton("Half (500)")
        btn_l_close = QPushButton("Close (1000)")
        btn_l_open.clicked.connect(lambda: self.left_slider.setValue(0))
        btn_l_half.clicked.connect(lambda: self.left_slider.setValue(500))
        btn_l_close.clicked.connect(lambda: self.left_slider.setValue(1000))
        left_ctrl_layout.addWidget(btn_l_open, 2, 0)
        left_ctrl_layout.addWidget(btn_l_half, 2, 1)
        left_ctrl_layout.addWidget(btn_l_close, 2, 2)
        btn_l_stop = QPushButton("Emergency Open")
        btn_l_stop.clicked.connect(lambda: self.left_slider.setValue(0))
        left_ctrl_layout.addWidget(btn_l_stop, 2, 3)
        left_col.addWidget(left_ctrl)

        right_ctrl = QGroupBox("Right Hand Control")
        right_ctrl_layout = QGridLayout(right_ctrl)
        right_ctrl_layout.addWidget(QLabel("Mode"), 0, 0)
        self.right_mode_combo = QComboBox()
        self.right_mode_combo.addItems(["position", "angle"])
        right_ctrl_layout.addWidget(self.right_mode_combo, 0, 1)
        right_ctrl_layout.addWidget(QLabel("Grasp Slider"), 1, 0)
        self.right_slider = QSlider(Qt.Horizontal)
        self.right_slider.setRange(0, 1000)
        self.right_slider.setValue(0)
        right_ctrl_layout.addWidget(self.right_slider, 1, 1, 1, 2)
        self.right_slider_label = QLabel("0")
        right_ctrl_layout.addWidget(self.right_slider_label, 1, 3)
        self.right_slider.valueChanged.connect(lambda v: self.right_slider_label.setText(str(v)))
        btn_r_open = QPushButton("Open (0)")
        btn_r_half = QPushButton("Half (500)")
        btn_r_close = QPushButton("Close (1000)")
        btn_r_open.clicked.connect(lambda: self.right_slider.setValue(0))
        btn_r_half.clicked.connect(lambda: self.right_slider.setValue(500))
        btn_r_close.clicked.connect(lambda: self.right_slider.setValue(1000))
        right_ctrl_layout.addWidget(btn_r_open, 2, 0)
        right_ctrl_layout.addWidget(btn_r_half, 2, 1)
        right_ctrl_layout.addWidget(btn_r_close, 2, 2)
        btn_r_stop = QPushButton("Emergency Open")
        btn_r_stop.clicked.connect(lambda: self.right_slider.setValue(0))
        right_ctrl_layout.addWidget(btn_r_stop, 2, 3)
        right_col.addWidget(right_ctrl)

        left_status = QGroupBox("Left Current (Latest)")
        left_status_layout = QGridLayout(left_status)
        left_status_layout.addWidget(QLabel("Joint"), 0, 0)
        left_status_layout.addWidget(QLabel("Value"), 0, 1)
        for i, name in enumerate(JOINT_NAMES):
            left_status_layout.addWidget(QLabel(name), i + 1, 0)
            ll = QLabel("-")
            self.left_labels.append(ll)
            left_status_layout.addWidget(ll, i + 1, 1)
        left_col.addWidget(left_status)

        right_status = QGroupBox("Right Current (Latest)")
        right_status_layout = QGridLayout(right_status)
        right_status_layout.addWidget(QLabel("Joint"), 0, 0)
        right_status_layout.addWidget(QLabel("Value"), 0, 1)
        for i, name in enumerate(JOINT_NAMES):
            right_status_layout.addWidget(QLabel(name), i + 1, 0)
            rr = QLabel("-")
            self.right_labels.append(rr)
            right_status_layout.addWidget(rr, i + 1, 1)
        right_col.addWidget(right_status)

        if self.no_plot:
            left_col.addWidget(QLabel("Live plot disabled by --no-plot"))
            right_col.addWidget(QLabel("Live plot disabled by --no-plot"))
            self.plot_l = None
            self.plot_r = None
        else:
            self.plot_l = HandPlotCanvas("Left", history_len=self.history_len, dt_s=self.dt_s)
            self.plot_r = HandPlotCanvas("Right", history_len=self.history_len, dt_s=self.dt_s)
            left_col.addWidget(self.plot_l)
            right_col.addWidget(self.plot_r)

        main.addLayout(left_col, 1)
        main.addLayout(right_col, 1)
        self.setCentralWidget(root)

    def _tick(self):
        self.hand_l.send_grasp(self.left_slider.value(), mode=self.left_mode_combo.currentText())
        self.hand_r.send_grasp(self.right_slider.value(), mode=self.right_mode_combo.currentText())

        state_l = self.hand_l.read_state()
        state_r = self.hand_r.read_state()
        curr_l = [0.0] * 6 if state_l is None else list(state_l.current)
        curr_r = [0.0] * 6 if state_r is None else list(state_r.current)
        force_l = [0.0] * 6 if state_l is None else list(state_l.force_act)
        force_r = [0.0] * 6 if state_r is None else list(state_r.force_act)
        temp_l = [0.0] * 6 if state_l is None else list(state_l.temperature)
        temp_r = [0.0] * 6 if state_r is None else list(state_r.temperature)

        for i in range(6):
            self.current_left_hist[i].append(float(curr_l[i]))
            self.current_right_hist[i].append(float(curr_r[i]))
            self.force_left_hist[i].append(float(force_l[i]))
            self.force_right_hist[i].append(float(force_r[i]))
            self.temp_left_hist[i].append(float(temp_l[i]))
            self.temp_right_hist[i].append(float(temp_r[i]))
            self.left_labels[i].setText(str(curr_l[i]))
            self.right_labels[i].setText(str(curr_r[i]))

        self.tick_count += 1
        if self.tick_count % 2 == 0:
            if self.plot_l is not None:
                self.plot_l.update_plot(
                    self.current_left_hist,
                    self.force_left_hist,
                    self.temp_left_hist,
                )
            if self.plot_r is not None:
                self.plot_r.update_plot(
                    self.current_right_hist,
                    self.force_right_hist,
                    self.temp_right_hist,
                )


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--network", default=None, help="DDS interface, e.g. enp39s0")
    p.add_argument("--dt-ms", type=int, default=50, help="UI/control update interval in ms")
    p.add_argument("--no-plot", action="store_true", help="Disable live plotting (control + status only)")
    return p.parse_args()


def main():
    args = parse_args()
    # Workaround: some CycloneDDS builds crash with explicit interface config.
    # Use autodetect init path to avoid native buffer overflow in init.
    if args.network:
        print(
            f"[warn] --network {args.network} requested, but explicit interface init is disabled "
            "due to CycloneDDS crash. Using autodetect DDS interface."
        )
    ChannelFactoryInitialize(0)

    app = QApplication([])
    window = MainWindow(dt_ms=args.dt_ms, no_plot=args.no_plot)
    window.show()
    app.exec_()


if __name__ == "__main__":
    main()
