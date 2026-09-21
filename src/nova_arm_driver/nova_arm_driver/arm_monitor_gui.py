#!/usr/bin/env python3
"""
arm_monitor_gui.py  -  live monitor for the arm servos and both coordinators.

Run (workspace sourced, robot stack running):
    python3 arm_monitor_gui.py
    python3 arm_monitor_gui.py --window 300 --servos 1 2 3 4 5 6

Needs:  sudo apt install python3-tk python3-matplotlib

It is a plain Python program (not started with ros2 run). It creates a small
ROS 2 node in a background thread, only LISTENS, and has one button that calls
/arm_health/reset (to leave FAULT after a human has fixed the problem).

Listens to
  /arm_servo_status   diagnostic_msgs/DiagnosticArray  (from ArmDriver)
  /arm_health_state   /arm_health_detail   /arm_health_event   (health coordinator)
  /arm_motion_allowed std_msgs/Bool
  /presenter_state    std_msgs/String                    (presenter coordinator)
"""

import argparse
import json
import queue
import sys
import threading
import time
import tkinter as tk
from collections import deque
from tkinter import messagebox, ttk

import matplotlib

matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg  # noqa: E402
from matplotlib.figure import Figure  # noqa: E402

import rclpy  # noqa: E402
from diagnostic_msgs.msg import DiagnosticArray  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy  # noqa: E402
from std_msgs.msg import Bool, String  # noqa: E402
from std_srvs.srv import Trigger  # noqa: E402

ERR_NAMES = {0: "voltage", 1: "angle", 2: "OVERHEAT", 3: "range",
             4: "checksum", 5: "OVERLOAD", 6: "instruction"}
WARN_TEMP = 55
RECOVER_TEMP = 50
VOLT_MIN, VOLT_MAX = 95, 125
SERVO_COLORS = ["#1f77b4", "#ff7f0e", "#2ca02c", "#d62728",
                "#9467bd", "#8c564b", "#e377c2", "#7f7f7f"]

HEALTH_COLORS = {"HEALTHY": "#2e9e4f", "DIAGNOSTIC": "#e08a00",
                 "RECOVERING": "#e08a00", "VERIFYING": "#c9a800",
                 "FAULT": "#c62828"}
PRESENTER_COLORS = {"IDLE": "#6b7280", "DRIVE": "#1f6fd1", "SCAN": "#1f6fd1",
                    "SPEAK": "#1f6fd1", "ARM_PAUSED": "#e08a00",
                    "ARM_FAULT": "#c62828"}
EVENT_COLORS = {"ARM_PROBLEM_DETECTED": "#c62828", "ARM_RECOVERING": "#e08a00",
                "ARM_RECOVERED": "#2e9e4f", "ARM_PROBLEM_CLEARED": "#2e9e4f",
                "ARM_FAULT": "#000000"}
NO_DATA_COLOR = "#9ca3af"
STALE_AFTER_S = 3.5


def decode_err(err):
    return [n for b, n in ERR_NAMES.items() if err & (1 << b)]


def ticks_to_deg(v):
    return v * 300.0 / 1023.0 - 150.0


# --------------------------------------------------------------------- ROS side

class RosBridge:
    """Background ROS 2 node. Everything it hears is pushed onto self.q."""

    def __init__(self):
        rclpy.init()
        self.node = Node("arm_monitor_gui")
        self.q = queue.Queue()
        n = self.node
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)

        n.create_subscription(DiagnosticArray, "/arm_servo_status", self._servo, 10)
        n.create_subscription(String, "/arm_health_state",
                              lambda m: self._put("health", m.data), latched)
        n.create_subscription(String, "/arm_health_detail",
                              lambda m: self._put("detail", m.data), latched)
        n.create_subscription(String, "/arm_health_event",
                              lambda m: self._put("event", m.data), 10)
        n.create_subscription(Bool, "/arm_motion_allowed",
                              lambda m: self._put("motion", bool(m.data)), latched)
        n.create_subscription(String, "/presenter_state",
                              lambda m: self._put("presenter", m.data), 10)
        self.reset_client = n.create_client(Trigger, "/arm_health/reset")

        self.executor = SingleThreadedExecutor()
        self.executor.add_node(n)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()

    def _put(self, kind, payload):
        self.q.put((kind, time.time(), payload))

    def _servo(self, msg):
        out = {}
        for st in msg.status:
            try:
                sid = int(st.hardware_id)
            except ValueError:
                continue
            kv = {k.key: k.value for k in st.values}
            if kv.get("comm_ok") != "1":
                out[sid] = None
                continue
            try:
                out[sid] = {k: int(v) for k, v in kv.items()}
            except ValueError:
                out[sid] = None
        self._put("servo", out)

    def call_reset(self):
        if not self.reset_client.service_is_ready():
            self._put("log", ("Reset service /arm_health/reset not available.", "bad"))
            return
        fut = self.reset_client.call_async(Trigger.Request())

        def done(f):
            try:
                r = f.result()
                self._put("log", (f"Reset: {'OK' if r.success else 'refused'} - {r.message}",
                                  "ok" if r.success else "warn"))
            except Exception as exc:  # noqa: BLE001
                self._put("log", (f"Reset call failed: {exc}", "bad"))

        fut.add_done_callback(done)

    def shutdown(self):
        try:
            self.executor.shutdown()
            self.node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:  # noqa: BLE001
            pass


# ----------------------------------------------------------------------- GUI

class App:
    COLS = [("id", "ID", 40), ("torque", "Torque", 60), ("pos", "Pos °", 70),
            ("goal", "Goal °", 70), ("temp", "Temp °C", 70), ("volt", "Volt V", 65),
            ("load", "Load %", 65), ("tlim", "TorqueLim", 80), ("maxt", "MaxTorque", 80),
            ("flags", "HW error flags", 220)]

    def __init__(self, root, bridge, ids, window):
        self.root, self.bridge, self.ids, self.window = root, bridge, ids, window
        root.title("Nova arm monitor")
        root.geometry("1350x950")

        self.hist = {i: deque(maxlen=6000) for i in ids}   # (t, temp, load, tlim, pos)
        self.latest = {}
        self.servo_rx = 0.0
        self.health = None
        self.health_rx = 0.0
        self.events = deque(maxlen=200)                    # (t, name)
        self.marker_artists = []

        self._build_top()
        self._build_table()
        self._build_charts()

        root.protocol("WM_DELETE_WINDOW", self.close)
        self.poll()
        self.refresh_charts()

    # ---- layout --------------------------------------------------------

    def _build_top(self):
        top = ttk.Frame(self.root, padding=6)
        top.pack(fill="x")

        hf = ttk.LabelFrame(top, text="Arm health coordinator", padding=6)
        hf.pack(side="left", fill="y", padx=(0, 6))
        self.health_lbl = tk.Label(hf, text="NO SIGNAL", width=14, fg="white",
                                   bg=NO_DATA_COLOR, font=("Helvetica", 20, "bold"))
        self.health_lbl.pack(fill="x")
        self.reason_var = tk.StringVar(value="-")
        self.attempts_var = tk.StringVar(value="Attempts: -")
        self.motion_var = tk.StringVar(value="Arm motion allowed: -")
        self.event_var = tk.StringVar(value="Last event: -")
        ttk.Label(hf, textvariable=self.reason_var, wraplength=300).pack(anchor="w", pady=(4, 0))
        ttk.Label(hf, textvariable=self.attempts_var).pack(anchor="w")
        ttk.Label(hf, textvariable=self.motion_var).pack(anchor="w")
        ttk.Label(hf, textvariable=self.event_var).pack(anchor="w")
        ttk.Button(hf, text="Reset FAULT (after human fix)",
                   command=self.on_reset).pack(fill="x", pady=(6, 0))

        pf = ttk.LabelFrame(top, text="Presenter coordinator", padding=6)
        pf.pack(side="left", fill="y", padx=(0, 6))
        self.presenter_lbl = tk.Label(pf, text="NO SIGNAL", width=14, fg="white",
                                      bg=NO_DATA_COLOR, font=("Helvetica", 20, "bold"))
        self.presenter_lbl.pack(fill="x")
        self.servo_data_var = tk.StringVar(value="Servo data: waiting...")
        ttk.Label(pf, textvariable=self.servo_data_var).pack(anchor="w", pady=(4, 0))

        lf = ttk.LabelFrame(top, text="Event log", padding=4)
        lf.pack(side="left", fill="both", expand=True)
        self.log = tk.Text(lf, height=9, wrap="word", state="disabled")
        sb = ttk.Scrollbar(lf, command=self.log.yview)
        self.log.configure(yscrollcommand=sb.set)
        sb.pack(side="right", fill="y")
        self.log.pack(fill="both", expand=True)
        for tag, color in (("ok", "#1b7f3b"), ("warn", "#b26a00"),
                           ("bad", "#c62828"), ("info", "#111827")):
            self.log.tag_configure(tag, foreground=color)

    def _build_table(self):
        frame = ttk.LabelFrame(self.root, text="Servos (from /arm_servo_status)", padding=4)
        frame.pack(fill="x", padx=6)
        self.tree = ttk.Treeview(frame, columns=[c[0] for c in self.COLS],
                                 show="headings", height=len(self.ids))
        for key, title, width in self.COLS:
            self.tree.heading(key, text=title)
            self.tree.column(key, width=width, anchor="center" if key != "flags" else "w")
        self.tree.tag_configure("ok", background="#e8f5e9")
        self.tree.tag_configure("warn", background="#fff8dc")
        self.tree.tag_configure("bad", background="#fde0dc")
        self.tree.tag_configure("none", background="#eeeeee")
        self.tree.pack(fill="x")
        for i in self.ids:
            self.tree.insert("", "end", iid=str(i),
                             values=(i,) + ("--",) * (len(self.COLS) - 1), tags=("none",))

    def _build_charts(self):
        frame = ttk.Frame(self.root, padding=4)
        frame.pack(fill="both", expand=True)
        self.fig = Figure(figsize=(12, 5.5), dpi=90)
        specs = [("temp", "Temperature (°C)"), ("load", "Load (%)"),
                 ("tlim", "Torque limit"), ("pos", "Position (°)")]
        self.axes, self.lines = {}, {}
        for n, (key, title) in enumerate(specs, start=1):
            ax = self.fig.add_subplot(2, 2, n)
            ax.set_title(title, fontsize=10)
            ax.set_xlim(-self.window, 0)
            ax.grid(True, alpha=0.3)
            ax.tick_params(labelsize=8)
            self.axes[key] = ax
            self.lines[key] = {}
            for k, sid in enumerate(self.ids):
                (ln,) = ax.plot([], [], lw=1.6, label=f"ID {sid}",
                                color=SERVO_COLORS[k % len(SERVO_COLORS)])
                self.lines[key][sid] = ln
        self.axes["temp"].axhline(RECOVER_TEMP, color="#888", ls="--", lw=0.9)
        self.axes["temp"].axhline(WARN_TEMP, color="#c62828", ls="--", lw=0.9)
        self.axes["load"].set_ylim(0, 100)
        self.axes["tlim"].set_ylim(-50, 1100)
        self.axes["pos"].set_ylim(-155, 155)
        self.axes["temp"].legend(loc="upper left", fontsize=7, ncol=3)
        self.axes["pos"].set_xlabel("seconds ago", fontsize=8)
        self.axes["tlim"].set_xlabel("seconds ago", fontsize=8)
        self.fig.tight_layout()
        self.canvas = FigureCanvasTkAgg(self.fig, master=frame)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)

    # ---- helpers ---------------------------------------------------------

    def add_log(self, text, tag="info", t=None):
        stamp = time.strftime("%H:%M:%S", time.localtime(t or time.time()))
        self.log.configure(state="normal")
        self.log.insert("end", f"{stamp}  {text}\n", tag)
        self.log.see("end")
        self.log.configure(state="disabled")

    @staticmethod
    def set_badge(label, text, color):
        label.configure(text=text, bg=color)

    def on_reset(self):
        if messagebox.askyesno(
                "Reset FAULT",
                "Only reset after the cause has been fixed and the arm is SUPPORTED.\n\n"
                "The health coordinator will diagnose again and may restore servo torque.\n"
                "Continue?"):
            self.add_log("Reset requested from GUI.", "warn")
            self.bridge.call_reset()

    # ---- incoming data ------------------------------------------------------

    def poll(self):
        got_servo = False
        try:
            while True:
                kind, t, payload = self.bridge.q.get_nowait()
                if kind == "servo":
                    self.handle_servo(t, payload)
                    got_servo = True
                elif kind == "health":
                    self.handle_health(t, payload)
                elif kind == "detail":
                    self.handle_detail(payload)
                elif kind == "event":
                    self.events.append((t, payload))
                    self.event_var.set(f"Last event: {payload}")
                    self.add_log(f"EVENT {payload}",
                                 "bad" if payload in ("ARM_FAULT", "ARM_PROBLEM_DETECTED")
                                 else "ok" if payload in ("ARM_RECOVERED", "ARM_PROBLEM_CLEARED")
                                 else "warn", t)
                elif kind == "motion":
                    self.motion_var.set(f"Arm motion allowed: {'YES' if payload else 'NO'}")
                elif kind == "presenter":
                    self.set_badge(self.presenter_lbl, payload,
                                   PRESENTER_COLORS.get(payload, "#6b7280"))
                    self.add_log(f"Presenter: {payload}", "info", t)
                elif kind == "log":
                    self.add_log(payload[0], payload[1], t)
        except queue.Empty:
            pass

        if got_servo:
            self.refresh_table()

        now = time.time()
        if self.health is not None and now - self.health_rx > STALE_AFTER_S:
            self.set_badge(self.health_lbl, "NO SIGNAL", NO_DATA_COLOR)
        if self.servo_rx:
            age = now - self.servo_rx
            self.servo_data_var.set("Servo data: OK" if age < STALE_AFTER_S
                                    else f"Servo data: STALE ({age:.0f} s)")
        self.root.after(200, self.poll)

    def handle_servo(self, t, data):
        self.servo_rx = t
        nan = float("nan")
        for sid in self.ids:
            d = data.get(sid)
            self.latest[sid] = d
            if d is None:
                self.hist[sid].append((t, nan, nan, nan, nan))
            else:
                self.hist[sid].append((t, d["temp"], (d["load"] & 0x3FF) * 100.0 / 1023.0,
                                       d["torque_limit"], ticks_to_deg(d["position"])))

    def handle_health(self, t, state):
        self.health_rx = t
        if state != self.health:
            self.add_log(f"Arm health: {self.health or '?'} -> {state}",
                         "bad" if state == "FAULT" else "ok" if state == "HEALTHY" else "warn", t)
            self.health = state
        self.set_badge(self.health_lbl, state, HEALTH_COLORS.get(state, "#6b7280"))

    def handle_detail(self, raw):
        try:
            d = json.loads(raw)
        except ValueError:
            return
        self.reason_var.set(d.get("reason") or "-")
        att = d.get("attempts") or {}
        self.attempts_var.set("Attempts: " + (", ".join(f"ID {k}: {v}" for k, v in att.items())
                                              if att else "none"))

    # ---- table / charts ---------------------------------------------------------

    def refresh_table(self):
        for sid in self.ids:
            d = self.latest.get(sid)
            if d is None:
                self.tree.item(str(sid), values=(sid, "NO RESPONSE") + ("--",) * 8, tags=("bad",))
                continue
            flags = decode_err(d["err"])
            volt = d["voltage"]
            bad = (d["torque_limit"] < d["max_torque"] or d["max_torque"] == 0
                   or d["torque_enable"] == 0 or bool(flags))
            warn = d["temp"] >= WARN_TEMP or not (VOLT_MIN <= volt <= VOLT_MAX)
            tag = "bad" if bad else "warn" if warn else "ok"
            self.tree.item(str(sid), values=(
                sid, "ON" if d["torque_enable"] else "OFF",
                f"{ticks_to_deg(d['position']):.1f}", f"{ticks_to_deg(d['goal']):.1f}",
                d["temp"], f"{volt / 10:.1f}", f"{(d['load'] & 0x3FF) * 100.0 / 1023.0:.0f}",
                d["torque_limit"], d["max_torque"], ", ".join(flags) if flags else "-"),
                tags=(tag,))

    def refresh_charts(self):
        now = time.time()
        idx = {"temp": 1, "load": 2, "tlim": 3, "pos": 4}
        temps = []
        for key, col in idx.items():
            for sid in self.ids:
                xs, ys = [], []
                for row in self.hist[sid]:
                    if now - row[0] <= self.window:
                        xs.append(row[0] - now)
                        ys.append(row[col])
                self.lines[key][sid].set_data(xs, ys)
                if key == "temp":
                    temps += [y for y in ys if y == y]
        lo = min(temps + [20]) - 2
        hi = max(temps + [WARN_TEMP]) + 3
        self.axes["temp"].set_ylim(lo, hi)

        for art in self.marker_artists:
            try:
                art.remove()
            except Exception:  # noqa: BLE001
                pass
        self.marker_artists = []
        for t, name in self.events:
            if now - t <= self.window:
                for ax in self.axes.values():
                    self.marker_artists.append(ax.axvline(
                        t - now, color=EVENT_COLORS.get(name, "#444"), ls=":", lw=1.4))

        for ax in self.axes.values():
            ax.set_xlim(-self.window, 0)
        self.canvas.draw_idle()
        self.root.after(1000, self.refresh_charts)

    def close(self):
        self.bridge.shutdown()
        self.root.destroy()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--window", type=float, default=120.0, help="chart window in seconds")
    ap.add_argument("--servos", type=int, nargs="+", default=[1, 2, 3, 4, 5, 6])
    args = ap.parse_args()

    bridge = RosBridge()
    root = tk.Tk()
    App(root, bridge, args.servos, args.window)
    try:
        root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        bridge.shutdown()


if __name__ == "__main__":
    sys.exit(main())