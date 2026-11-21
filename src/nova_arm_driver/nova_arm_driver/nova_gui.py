#!/usr/bin/env python3
"""
Nova Arm Control Center
=======================

One window that replaces nova_teleop + nova_pose_manager and can run
mission sequences like nova_demo:

  * Jog joints with sliders / spin boxes or the gamepad (/joy)
  * Save / update / delete / move to poses in ~/nova_arm_ws/poses.yaml
  * Build and run multi-pose sequences with pauses (smooth cubic splines)
  * Speed control (/arm_speed), motion enable, and a STOP button (Esc)

This node is the ONLY publisher on /arm_command while it runs. A single
50 Hz timer streams either the jog target or the active trajectory, so
the jog stream and trajectory never fight each other.
"""

import os
import signal
import sys
import threading
import time

import numpy as np
import yaml
from scipy.interpolate import CubicSpline

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState, Joy
from std_msgs.msg import Int32

from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import (
    QAbstractItemView,
    QApplication,
    QDoubleSpinBox,
    QFrame,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QListWidget,
    QListWidgetItem,
    QMessageBox,
    QPlainTextEdit,
    QProgressBar,
    QPushButton,
    QShortcut,
    QSlider,
    QSplitter,
    QTabWidget,
    QVBoxLayout,
    QWidget,
)
from PyQt5.QtGui import QKeySequence


# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

POSE_FILE = os.path.expanduser("~/nova_arm_ws/poses.yaml")

JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "gripper"]

# ArmDriver clamps to +/-150 deg (+/-2.618 rad); stay just inside that.
JOINT_LIMIT = 2.6

RATE_HZ = 50.0
DT = 1.0 / RATE_HZ

# Gamepad jog speeds in rad/s at full stick deflection.
# (Old teleop: 0.05 rad per /joy msg at ~20 Hz = ~1.0 rad/s, fine 0.2 rad/s.)
JOG_SPEED = 1.0
FINE_JOG_SPEED = 0.2
DEADZONE = 0.10
JOY_TIMEOUT = 0.5  # s without /joy -> stop jogging

# Same as nova_demo.run_full_mission(): stop + 1 s at grasp and place.
DEMO_SEQUENCE = [
    ("home", 0.0),
    ("ready", 0.0),
    ("approach", 0.0),
    ("grasp", 1.0),
    ("carry", 0.0),
    ("place", 1.0),
    ("home", 0.0),
]

PAGES = ["Joint1 / Joint2", "Joint3 / Joint4", "Joint5 / Gripper"]

SLIDER_SCALE = 1000  # slider ticks per radian


# ---------------------------------------------------------------------------
# Pose file helpers
# ---------------------------------------------------------------------------

def load_poses():
    if not os.path.exists(POSE_FILE):
        return {}
    with open(POSE_FILE, "r") as f:
        data = yaml.safe_load(f)
    return data or {}


def save_poses(poses):
    os.makedirs(os.path.dirname(POSE_FILE), exist_ok=True)
    tmp = POSE_FILE + ".tmp"
    with open(tmp, "w") as f:
        yaml.dump(poses, f, sort_keys=False)
    os.replace(tmp, POSE_FILE)  # atomic: never leaves a half-written file


def pose_to_vector(pose):
    """Return positions ordered as JOINT_NAMES, whatever order the file uses."""
    names = list(pose.get("names", JOINT_NAMES))
    positions = list(pose["positions"])
    lookup = dict(zip(names, positions))
    missing = [j for j in JOINT_NAMES if j not in lookup]
    if missing:
        raise ValueError(f"pose is missing joints: {', '.join(missing)}")
    return np.array([float(lookup[j]) for j in JOINT_NAMES])


def vector_to_pose(vec):
    return {
        "names": list(JOINT_NAMES),
        "positions": [round(float(v), 4) for v in vec],
    }


# ---------------------------------------------------------------------------
# ROS node
# ---------------------------------------------------------------------------

class ArmControlNode(Node):
    """Owns /arm_command. All shared state is guarded by self.lock."""

    def __init__(self):
        super().__init__("nova_gui")

        self.lock = threading.Lock()

        self.cmd_pub = self.create_publisher(JointState, "/arm_command", 10)
        self.speed_pub = self.create_publisher(Int32, "/arm_speed", 10)
        self.create_subscription(Joy, "/joy", self._joy_cb, 10)

        self.target = np.zeros(len(JOINT_NAMES))
        self.output_enabled = False

        # Gamepad state
        self.page = 0
        self.fine_mode = False
        self._axes = []
        self._prev_buttons = []
        self._last_joy = 0.0

        # Trajectory state
        self._plan = []
        self._plan_index = 0
        self._seg_start = 0.0
        self._plan_start = 0.0
        self._plan_total = 0.0
        self.status = "Idle"

        self._events = []

        self.create_timer(DT, self._tick)

    # ---------------- callbacks (executor thread) ----------------

    def _joy_cb(self, msg):
        with self.lock:
            buttons = list(msg.buttons)
            prev = self._prev_buttons or [0] * len(buttons)

            def pressed(i):
                return i < len(buttons) and buttons[i] and not (i < len(prev) and prev[i])

            if pressed(4):  # L1
                self.page = (self.page - 1) % 3
            if pressed(5):  # R1
                self.page = (self.page + 1) % 3
            if pressed(2):  # X
                self.fine_mode = not self.fine_mode

            self._prev_buttons = buttons
            self._axes = list(msg.axes)
            self._last_joy = time.monotonic()

    def _tick(self):
        now = time.monotonic()
        with self.lock:
            if self._plan:
                self._advance_plan(now)
            else:
                self._apply_jog(now)

            if not self.output_enabled:
                return
            positions = self.target.tolist()

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(JOINT_NAMES)
        msg.position = positions
        self.cmd_pub.publish(msg)

    def _apply_jog(self, now):
        if now - self._last_joy > JOY_TIMEOUT or len(self._axes) < 4:
            return

        def dz(v):
            return 0.0 if abs(v) < DEADZONE else v

        left = -dz(self._axes[1])
        right = -dz(self._axes[3])
        if left == 0.0 and right == 0.0:
            return

        speed = FINE_JOG_SPEED if self.fine_mode else JOG_SPEED
        j1 = self.page * 2
        self.target[j1] += left * speed * DT
        self.target[j1 + 1] += right * speed * DT
        np.clip(self.target, -JOINT_LIMIT, JOINT_LIMIT, out=self.target)

    def _advance_plan(self, now):
        seg = self._plan[self._plan_index]
        t = now - self._seg_start

        if t < seg["duration"]:
            self.target = np.clip(seg["spline"](t), -JOINT_LIMIT, JOINT_LIMIT)
            self.status = f"Moving to '{seg['label']}'"
            return

        self.target = seg["end"].copy()

        if t < seg["duration"] + seg["dwell"]:
            self.status = f"Pausing at '{seg['label']}'"
            return

        # Segment finished
        self._seg_start += seg["duration"] + seg["dwell"]
        self._plan_index += 1
        if self._plan_index >= len(self._plan):
            self._plan = []
            self.status = "Idle"
            self._events.append(("ok", "Motion complete."))

    # ---------------- API for the GUI thread ----------------

    def get_state(self):
        with self.lock:
            now = time.monotonic()
            running = bool(self._plan)
            progress = 0.0
            if running and self._plan_total > 0:
                progress = min(1.0, (now - self._plan_start) / self._plan_total)
            return {
                "target": self.target.copy(),
                "enabled": self.output_enabled,
                "page": self.page,
                "fine": self.fine_mode,
                "joy_alive": now - self._last_joy < JOY_TIMEOUT,
                "running": running,
                "progress": progress,
                "status": self.status,
            }

    def pop_events(self):
        with self.lock:
            events, self._events = self._events, []
            return events

    def set_joint(self, index, value):
        with self.lock:
            if self._plan:
                return
            self.target[index] = float(np.clip(value, -JOINT_LIMIT, JOINT_LIMIT))

    def set_all(self, vec):
        with self.lock:
            if self._plan:
                return
            self.target = np.clip(np.asarray(vec, float), -JOINT_LIMIT, JOINT_LIMIT)

    def set_output(self, enabled):
        with self.lock:
            self.output_enabled = enabled
            if not enabled:
                self._plan = []
                self.status = "Idle"

    def set_speed(self, percent):
        msg = Int32()
        msg.data = int(percent)
        self.speed_pub.publish(msg)

    def driver_online(self):
        try:
            return "arm_driver" in self.get_node_names()
        except Exception:
            return False

    def stop(self):
        with self.lock:
            was_running = bool(self._plan)
            self._plan = []
            self.status = "Stopped"
        return was_running

    def start_sequence(self, steps, seg_time):
        """
        steps: list of (label, positions, pause_s).
        Consecutive poses with no pause are blended into one smooth spline;
        a pause (or the last step) makes the arm come to rest there.
        """
        with self.lock:
            if not self.output_enabled:
                raise RuntimeError("Motion is disabled.")
            if self._plan:
                raise RuntimeError("A motion is already running.")

            plan = []
            pts = [self.target.copy()]
            for i, (label, pos, pause) in enumerate(steps):
                pts.append(np.clip(np.asarray(pos, float), -JOINT_LIMIT, JOINT_LIMIT))
                if pause > 0 or i == len(steps) - 1:
                    times = np.arange(len(pts)) * seg_time
                    spline = CubicSpline(
                        times, np.vstack(pts), axis=0, bc_type="clamped"
                    )
                    plan.append({
                        "spline": spline,
                        "duration": float(times[-1]),
                        "dwell": float(pause),
                        "end": pts[-1].copy(),
                        "label": label,
                    })
                    pts = [pts[-1]]

            now = time.monotonic()
            self._plan = plan
            self._plan_index = 0
            self._seg_start = now
            self._plan_start = now
            self._plan_total = sum(p["duration"] + p["dwell"] for p in plan)
            self.status = "Starting"


# ---------------------------------------------------------------------------
# GUI
# ---------------------------------------------------------------------------

STYLE = """
QWidget { background: #1e2127; color: #dcdfe4; font-size: 13px; }
QGroupBox { border: 1px solid #3a3f4b; border-radius: 6px; margin-top: 14px; padding: 8px; }
QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 4px; color: #8fa1b3; }
QPushButton { background: #2c313a; border: 1px solid #454b58; border-radius: 4px; padding: 6px 10px; }
QPushButton:hover { background: #353b45; }
QPushButton:disabled { color: #5c6370; border-color: #333842; }
QLineEdit, QDoubleSpinBox, QListWidget, QPlainTextEdit {
    background: #16181d; border: 1px solid #3a3f4b; border-radius: 4px; padding: 3px;
}
QListWidget::item:selected { background: #3d5a80; }
QTabWidget::pane { border: 1px solid #3a3f4b; border-radius: 4px; }
QTabBar::tab { background: #2c313a; padding: 6px 14px; border-top-left-radius: 4px; border-top-right-radius: 4px; }
QTabBar::tab:selected { background: #3d5a80; }
QProgressBar { border: 1px solid #3a3f4b; border-radius: 4px; text-align: center; background: #16181d; }
QProgressBar::chunk { background: #4c9f70; }
QSlider::groove:horizontal { height: 6px; background: #3a3f4b; border-radius: 3px; }
QSlider::handle:horizontal { background: #8fb8de; width: 14px; margin: -5px 0; border-radius: 7px; }
QSlider::handle:horizontal:disabled { background: #5c6370; }
#stopButton { background: #b33939; color: white; font-weight: bold; font-size: 16px; border: none; padding: 10px 22px; }
#stopButton:hover { background: #d14545; }
#enableButton { font-weight: bold; padding: 10px 16px; }
#enableButton:checked { background: #2e7d4f; border-color: #3fa66a; }
#jointRowActive { background: #283447; border-radius: 4px; }
"""


class JointRow(QFrame):
    def __init__(self, index, name, node):
        super().__init__()
        self.index = index
        self.node = node

        lay = QHBoxLayout(self)
        lay.setContentsMargins(6, 3, 6, 3)

        self.label = QLabel(name)
        self.label.setFixedWidth(70)

        self.slider = QSlider(Qt.Horizontal)
        self.slider.setRange(int(-JOINT_LIMIT * SLIDER_SCALE), int(JOINT_LIMIT * SLIDER_SCALE))
        self.slider.valueChanged.connect(self._slider_changed)

        self.spin = QDoubleSpinBox()
        self.spin.setRange(-JOINT_LIMIT, JOINT_LIMIT)
        self.spin.setDecimals(3)
        self.spin.setSingleStep(0.01)
        self.spin.setSuffix(" rad")
        self.spin.setKeyboardTracking(False)  # no jumps while typing "1.5"
        self.spin.setFixedWidth(110)
        self.spin.valueChanged.connect(self._spin_changed)

        self.deg = QLabel()
        self.deg.setFixedWidth(60)
        self.deg.setAlignment(Qt.AlignRight | Qt.AlignVCenter)

        lay.addWidget(self.label)
        lay.addWidget(self.slider, 1)
        lay.addWidget(self.spin)
        lay.addWidget(self.deg)

    def _slider_changed(self, v):
        self.node.set_joint(self.index, v / SLIDER_SCALE)

    def _spin_changed(self, v):
        self.node.set_joint(self.index, v)

    def show_value(self, value):
        if not self.slider.isSliderDown():
            self.slider.blockSignals(True)
            self.slider.setValue(int(round(value * SLIDER_SCALE)))
            self.slider.blockSignals(False)
        if not self.spin.hasFocus():
            self.spin.blockSignals(True)
            self.spin.setValue(value)
            self.spin.blockSignals(False)
        self.deg.setText(f"{np.degrees(value):6.1f}°")

    def set_active(self, active):
        self.setObjectName("jointRowActive" if active else "")
        self.style().unpolish(self)
        self.style().polish(self)


class MainWindow(QWidget):

    def __init__(self, node):
        super().__init__()
        self.node = node
        self.poses = {}
        self.sequence = []  # list of [name, pause]

        self.setWindowTitle("Nova Arm Control Center")
        self.resize(1100, 760)
        self.setStyleSheet(STYLE)

        root = QVBoxLayout(self)
        root.addLayout(self._build_top_bar())

        splitter = QSplitter(Qt.Horizontal)
        splitter.addWidget(self._build_jog_panel())
        splitter.addWidget(self._build_right_tabs())
        splitter.setSizes([560, 540])

        vsplit = QSplitter(Qt.Vertical)
        vsplit.addWidget(splitter)
        vsplit.addWidget(self._build_log())
        vsplit.setSizes([600, 140])
        root.addWidget(vsplit, 1)

        QShortcut(QKeySequence(Qt.Key_Escape), self, activated=self.on_stop)

        self.reload_poses()
        if "home" in self.poses:
            try:
                self.node.set_all(pose_to_vector(self.poses["home"]))
                self.log("Jog target initialised to 'home'.")
            except ValueError as e:
                self.log(f"'home' pose invalid: {e}", "warn")

        self.poll_timer = QTimer(self)
        self.poll_timer.timeout.connect(self.refresh)
        self.poll_timer.start(50)

        self.driver_timer = QTimer(self)
        self.driver_timer.timeout.connect(self.refresh_driver_status)
        self.driver_timer.start(1000)
        self.refresh_driver_status()

        self.log("Ready. Motion is DISABLED until you press 'Enable Motion'.")

    # ---------------- layout builders ----------------

    def _build_top_bar(self):
        bar = QHBoxLayout()

        title = QLabel("<b style='font-size:18px'>Nova Arm</b>")
        bar.addWidget(title)
        bar.addSpacing(16)

        self.driver_label = QLabel()
        bar.addWidget(self.driver_label)
        bar.addSpacing(16)

        self.status_label = QLabel()
        bar.addWidget(self.status_label, 1)

        bar.addWidget(QLabel("Speed"))
        self.speed_slider = QSlider(Qt.Horizontal)
        self.speed_slider.setRange(5, 100)
        self.speed_slider.setValue(30)
        self.speed_slider.setFixedWidth(160)
        self.speed_label = QLabel("30%")
        self.speed_label.setFixedWidth(40)
        self.speed_slider.valueChanged.connect(lambda v: self.speed_label.setText(f"{v}%"))
        self.speed_slider.sliderReleased.connect(self.on_speed_released)
        bar.addWidget(self.speed_slider)
        bar.addWidget(self.speed_label)
        bar.addSpacing(12)

        self.enable_button = QPushButton("Enable Motion")
        self.enable_button.setObjectName("enableButton")
        self.enable_button.setCheckable(True)
        self.enable_button.clicked.connect(self.on_enable_clicked)
        bar.addWidget(self.enable_button)

        self.stop_button = QPushButton("STOP  (Esc)")
        self.stop_button.setObjectName("stopButton")
        self.stop_button.clicked.connect(self.on_stop)
        bar.addWidget(self.stop_button)

        return bar

    def _build_jog_panel(self):
        box = QGroupBox("Jog")
        lay = QVBoxLayout(box)

        self.gamepad_label = QLabel()
        lay.addWidget(self.gamepad_label)

        self.rows = []
        for i, name in enumerate(JOINT_NAMES):
            row = JointRow(i, name, self.node)
            self.rows.append(row)
            lay.addWidget(row)

        btns = QHBoxLayout()
        zero = QPushButton("Zero all joints")
        zero.clicked.connect(self.on_zero)
        btns.addWidget(zero)
        btns.addStretch(1)
        lay.addLayout(btns)

        help_lbl = QLabel(
            "<span style='color:#8a93a3'>Gamepad: L1/R1 = change joint pair, "
            "X = fine mode, sticks = jog. Sliders and boxes move the arm "
            "directly when motion is enabled.</span>"
        )
        help_lbl.setWordWrap(True)
        lay.addWidget(help_lbl)
        lay.addStretch(1)

        self.jog_widgets = [r for r in self.rows] + [zero]
        return box

    def _build_right_tabs(self):
        tabs = QTabWidget()
        tabs.addTab(self._build_pose_tab(), "Poses")
        tabs.addTab(self._build_sequence_tab(), "Sequence")
        return tabs

    def _build_pose_tab(self):
        w = QWidget()
        lay = QVBoxLayout(w)

        self.pose_list = QListWidget()
        self.pose_list.itemDoubleClicked.connect(lambda _: self.on_move_to())
        self.pose_list.currentItemChanged.connect(self.on_pose_selected)
        lay.addWidget(self.pose_list, 1)

        self.pose_detail = QLabel()
        self.pose_detail.setStyleSheet("color:#8a93a3; font-family: monospace;")
        self.pose_detail.setWordWrap(True)
        lay.addWidget(self.pose_detail)

        grid = QGridLayout()
        self.move_button = QPushButton("Move To")
        self.move_button.clicked.connect(self.on_move_to)
        self.update_button = QPushButton("Update with current")
        self.update_button.clicked.connect(self.on_update_pose)
        self.delete_button = QPushButton("Delete")
        self.delete_button.clicked.connect(self.on_delete_pose)
        reload_button = QPushButton("Reload file")
        reload_button.clicked.connect(self.reload_poses)

        self.move_time = QDoubleSpinBox()
        self.move_time.setRange(0.5, 20.0)
        self.move_time.setValue(2.0)
        self.move_time.setSuffix(" s")

        grid.addWidget(self.move_button, 0, 0)
        grid.addWidget(QLabel("Move time"), 0, 1, Qt.AlignRight)
        grid.addWidget(self.move_time, 0, 2)
        grid.addWidget(self.update_button, 1, 0)
        grid.addWidget(self.delete_button, 1, 1)
        grid.addWidget(reload_button, 1, 2)
        lay.addLayout(grid)

        save_row = QHBoxLayout()
        self.pose_name = QLineEdit()
        self.pose_name.setPlaceholderText("new pose name, e.g. pickup_bag")
        self.pose_name.returnPressed.connect(self.on_save_pose)
        save_button = QPushButton("Save current as new")
        save_button.clicked.connect(self.on_save_pose)
        save_row.addWidget(self.pose_name, 1)
        save_row.addWidget(save_button)
        lay.addLayout(save_row)

        file_lbl = QLabel(f"<span style='color:#5c6370'>{POSE_FILE}</span>")
        lay.addWidget(file_lbl)
        return w

    def _build_sequence_tab(self):
        w = QWidget()
        lay = QVBoxLayout(w)

        self.seq_list = QListWidget()
        self.seq_list.setSelectionMode(QAbstractItemView.SingleSelection)
        self.seq_list.currentRowChanged.connect(self.on_seq_selected)
        lay.addWidget(self.seq_list, 1)

        row1 = QHBoxLayout()
        add = QPushButton("+ Add selected pose")
        add.setToolTip("Adds the pose selected on the Poses tab")
        add.clicked.connect(self.on_seq_add)
        rem = QPushButton("Remove")
        rem.clicked.connect(self.on_seq_remove)
        up = QPushButton("▲")
        up.clicked.connect(lambda: self.on_seq_move(-1))
        down = QPushButton("▼")
        down.clicked.connect(lambda: self.on_seq_move(1))
        clear = QPushButton("Clear")
        clear.clicked.connect(self.on_seq_clear)
        for b in (add, rem, up, down, clear):
            row1.addWidget(b)
        lay.addLayout(row1)

        row2 = QHBoxLayout()
        row2.addWidget(QLabel("Pause at selected step"))
        self.pause_spin = QDoubleSpinBox()
        self.pause_spin.setRange(0.0, 30.0)
        self.pause_spin.setSingleStep(0.5)
        self.pause_spin.setSuffix(" s")
        self.pause_spin.valueChanged.connect(self.on_pause_changed)
        row2.addWidget(self.pause_spin)
        row2.addStretch(1)
        row2.addWidget(QLabel("Time per step"))
        self.seg_time = QDoubleSpinBox()
        self.seg_time.setRange(0.5, 20.0)
        self.seg_time.setValue(2.0)
        self.seg_time.setSuffix(" s")
        row2.addWidget(self.seg_time)
        lay.addLayout(row2)

        note = QLabel(
            "<span style='color:#8a93a3'>Steps without a pause are blended into "
            "one smooth spline. A pause makes the arm stop there (e.g. to grasp).</span>"
        )
        note.setWordWrap(True)
        lay.addWidget(note)

        row3 = QHBoxLayout()
        demo = QPushButton("Load demo mission")
        demo.clicked.connect(self.on_load_demo)
        self.run_button = QPushButton("▶  Run sequence")
        self.run_button.setStyleSheet("font-weight:bold;")
        self.run_button.clicked.connect(self.on_run_sequence)
        row3.addWidget(demo)
        row3.addStretch(1)
        row3.addWidget(self.run_button)
        lay.addLayout(row3)

        self.progress = QProgressBar()
        self.progress.setRange(0, 1000)
        self.progress.setFormat("")
        lay.addWidget(self.progress)

        self.seq_edit_widgets = [add, rem, up, down, clear, demo, self.pause_spin, self.seg_time]
        return w

    def _build_log(self):
        self.log_view = QPlainTextEdit()
        self.log_view.setReadOnly(True)
        self.log_view.setMaximumBlockCount(500)
        return self.log_view

    # ---------------- logging ----------------

    def log(self, text, level="info"):
        colors = {"info": "#dcdfe4", "ok": "#7ec699", "warn": "#e5c07b", "err": "#e06c75"}
        stamp = time.strftime("%H:%M:%S")
        self.log_view.appendHtml(
            f"<span style='color:#5c6370'>{stamp}</span> "
            f"<span style='color:{colors.get(level, '#dcdfe4')}'>{text}</span>"
        )

    # ---------------- periodic refresh ----------------

    def refresh(self):
        s = self.node.get_state()

        for i, row in enumerate(self.rows):
            row.show_value(float(s["target"][i]))
            row.set_active(s["joy_alive"] and i // 2 == s["page"])

        if s["joy_alive"]:
            mode = "FINE" if s["fine"] else "Normal"
            self.gamepad_label.setText(
                f"🎮 Gamepad active — <b>{PAGES[s['page']]}</b> — mode: <b>{mode}</b>"
            )
        else:
            self.gamepad_label.setText(
                f"<span style='color:#8a93a3'>🎮 Gamepad idle "
                f"(page: {PAGES[s['page']]}, mode: {'FINE' if s['fine'] else 'Normal'})</span>"
            )

        running = s["running"]
        enabled = s["enabled"]

        if not enabled:
            self.status_label.setText("<span style='color:#e5c07b'>● Motion disabled</span>")
        elif running:
            self.status_label.setText(f"<span style='color:#61afef'>● {s['status']}</span>")
        else:
            self.status_label.setText("<span style='color:#7ec699'>● Streaming — ready</span>")

        if self.enable_button.isChecked() != enabled:
            self.enable_button.setChecked(enabled)
        self.enable_button.setText("Motion ENABLED" if enabled else "Enable Motion")

        for wdg in self.jog_widgets:
            wdg.setEnabled(not running)
        for wdg in self.seq_edit_widgets:
            wdg.setEnabled(not running)
        can_move = enabled and not running
        self.move_button.setEnabled(can_move)
        self.run_button.setEnabled(can_move)
        self.update_button.setEnabled(not running)
        self.delete_button.setEnabled(not running)

        self.progress.setValue(int(s["progress"] * 1000) if running else 0)
        self.progress.setFormat(f"{s['progress'] * 100:.0f}%" if running else "")

        for level, text in self.node.pop_events():
            self.log(text, level)

    def refresh_driver_status(self):
        if self.node.driver_online():
            self.driver_label.setText("<span style='color:#7ec699'>● arm_driver online</span>")
        else:
            self.driver_label.setText("<span style='color:#e06c75'>● arm_driver not found</span>")

    # ---------------- top bar actions ----------------

    def on_enable_clicked(self, checked):
        if checked:
            vals = ", ".join(f"{v:+.2f}" for v in self.node.get_state()["target"])
            reply = QMessageBox.question(
                self,
                "Enable motion",
                "The arm will move to the joint values shown in the Jog panel:\n\n"
                f"[{vals}] rad\n\n"
                f"at {self.speed_slider.value()}% speed. Is the workspace clear?",
            )
            if reply != QMessageBox.Yes:
                self.enable_button.setChecked(False)
                return
            self.node.set_speed(self.speed_slider.value())
            self.node.set_output(True)
            self.log("Motion enabled.", "ok")
        else:
            self.node.set_output(False)
            self.log("Motion disabled. Commands are no longer sent.", "warn")

    def on_speed_released(self):
        v = self.speed_slider.value()
        self.node.set_speed(v)
        self.log(f"Speed set to {v}%.")

    def on_stop(self):
        if self.node.stop():
            self.log("STOP: trajectory aborted, holding current setpoint.", "err")
        else:
            self.log("STOP pressed (no trajectory running).", "warn")

    def on_zero(self):
        self.node.set_all(np.zeros(len(JOINT_NAMES)))
        self.log("All joint targets set to 0.")

    # ---------------- pose actions ----------------

    def reload_poses(self):
        try:
            self.poses = load_poses()
        except Exception as e:
            self.log(f"Failed to read {POSE_FILE}: {e}", "err")
            return
        current = self.selected_pose()
        self.pose_list.clear()
        for name in self.poses:
            self.pose_list.addItem(name)
        if current:
            items = self.pose_list.findItems(current, Qt.MatchExactly)
            if items:
                self.pose_list.setCurrentItem(items[0])
        self.render_sequence()

    def selected_pose(self):
        item = self.pose_list.currentItem()
        return item.text() if item else None

    def on_pose_selected(self, *_):
        name = self.selected_pose()
        if not name or name not in self.poses:
            self.pose_detail.setText("")
            return
        try:
            vec = pose_to_vector(self.poses[name])
            self.pose_detail.setText(
                "  ".join(f"{j}:{v:+.2f}" for j, v in zip(JOINT_NAMES, vec))
            )
        except Exception as e:
            self.pose_detail.setText(f"invalid pose: {e}")

    def _write_poses(self, action):
        try:
            save_poses(self.poses)
            self.log(action, "ok")
        except Exception as e:
            self.log(f"Failed to write {POSE_FILE}: {e}", "err")
        self.reload_poses()

    def on_save_pose(self):
        name = self.pose_name.text().strip()
        if not name:
            self.log("Enter a pose name first.", "warn")
            return
        if name in self.poses:
            r = QMessageBox.question(self, "Overwrite", f"Pose '{name}' exists. Overwrite?")
            if r != QMessageBox.Yes:
                return
        self.poses[name] = vector_to_pose(self.node.get_state()["target"])
        self.pose_name.clear()
        self._write_poses(f"Saved pose '{name}'.")
        items = self.pose_list.findItems(name, Qt.MatchExactly)
        if items:
            self.pose_list.setCurrentItem(items[0])

    def on_update_pose(self):
        name = self.selected_pose()
        if not name:
            return
        r = QMessageBox.question(
            self, "Update pose", f"Overwrite '{name}' with the current joint values?"
        )
        if r != QMessageBox.Yes:
            return
        self.poses[name] = vector_to_pose(self.node.get_state()["target"])
        self._write_poses(f"Updated pose '{name}'.")

    def on_delete_pose(self):
        name = self.selected_pose()
        if not name:
            return
        r = QMessageBox.question(self, "Delete pose", f"Delete pose '{name}'?")
        if r != QMessageBox.Yes:
            return
        del self.poses[name]
        self._write_poses(f"Deleted pose '{name}'.")

    def on_move_to(self):
        name = self.selected_pose()
        if not name:
            return
        self._run([(name, 0.0)], self.move_time.value())

    # ---------------- sequence actions ----------------

    def render_sequence(self):
        row = self.seq_list.currentRow()
        self.seq_list.clear()
        for i, (name, pause) in enumerate(self.sequence):
            text = f"{i + 1}.  {name}"
            if pause > 0:
                text += f"    ⏸ {pause:.1f} s"
            item = QListWidgetItem(text)
            if name not in self.poses:
                item.setForeground(Qt.red)
                item.setToolTip("Pose not found in poses.yaml")
            self.seq_list.addItem(item)
        if 0 <= row < len(self.sequence):
            self.seq_list.setCurrentRow(row)

    def on_seq_selected(self, row):
        self.pause_spin.blockSignals(True)
        self.pause_spin.setValue(self.sequence[row][1] if 0 <= row < len(self.sequence) else 0.0)
        self.pause_spin.blockSignals(False)

    def on_pause_changed(self, v):
        row = self.seq_list.currentRow()
        if 0 <= row < len(self.sequence):
            self.sequence[row][1] = v
            self.render_sequence()

    def on_seq_add(self):
        name = self.selected_pose()
        if not name:
            self.log("Select a pose on the Poses tab first.", "warn")
            return
        self.sequence.append([name, 0.0])
        self.render_sequence()
        self.seq_list.setCurrentRow(len(self.sequence) - 1)

    def on_seq_remove(self):
        row = self.seq_list.currentRow()
        if 0 <= row < len(self.sequence):
            del self.sequence[row]
            self.render_sequence()

    def on_seq_move(self, delta):
        row = self.seq_list.currentRow()
        new = row + delta
        if 0 <= row < len(self.sequence) and 0 <= new < len(self.sequence):
            self.sequence[row], self.sequence[new] = self.sequence[new], self.sequence[row]
            self.render_sequence()
            self.seq_list.setCurrentRow(new)

    def on_seq_clear(self):
        self.sequence = []
        self.render_sequence()

    def on_load_demo(self):
        self.sequence = [[n, p] for n, p in DEMO_SEQUENCE]
        self.render_sequence()
        missing = [n for n, _ in DEMO_SEQUENCE if n not in self.poses]
        if missing:
            self.log(f"Demo loaded, but missing poses: {', '.join(sorted(set(missing)))}", "warn")
        else:
            self.log("Demo mission loaded.")

    def on_run_sequence(self):
        if not self.sequence:
            self.log("Sequence is empty.", "warn")
            return
        self._run([(n, p) for n, p in self.sequence], self.seg_time.value())

    def _run(self, named_steps, seg_time):
        steps = []
        for name, pause in named_steps:
            if name not in self.poses:
                self.log(f"Pose '{name}' not found.", "err")
                return
            try:
                steps.append((name, pose_to_vector(self.poses[name]), pause))
            except ValueError as e:
                self.log(f"Pose '{name}': {e}", "err")
                return
        try:
            self.node.start_sequence(steps, seg_time)
        except RuntimeError as e:
            self.log(str(e), "warn")
            return
        label = " → ".join(n for n, _ in named_steps)
        self.log(f"Running: {label}")

    # ---------------- shutdown ----------------

    def closeEvent(self, event):
        self.node.stop()
        self.node.set_output(False)
        super().closeEvent(event)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = ArmControlNode()

    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    app = QApplication(sys.argv)
    window = MainWindow(node)
    window.show()

    # Let Ctrl+C in the terminal close the window.
    signal.signal(signal.SIGINT, lambda *_: app.quit())

    code = app.exec_()

    executor.shutdown()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    sys.exit(code)


if __name__ == "__main__":
    main()
