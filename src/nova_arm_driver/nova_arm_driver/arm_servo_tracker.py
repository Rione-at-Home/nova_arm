#!/usr/bin/env python3
"""
Simple desktop GUI for tracking per-servo status/notes across both arms.

Persists to a local JSON file (arm_servo_log.json, next to this script by
default) so your notes survive closing and reopening the app. No external
dependencies -- uses tkinter, which ships with standard Python installs on
Linux (may need `sudo apt install python3-tk` if it's not already present).

Run:
    python3 arm_servo_tracker.py
"""

import json
import os
import tkinter as tk
from tkinter import ttk
from datetime import datetime

STORAGE_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "arm_servo_log.json")

STATUS_OPTIONS = ["ok", "warn", "offline"]
STATUS_LABELS = {"ok": "OK", "warn": "Needs attention", "offline": "Offline"}
STATUS_COLORS = {"ok": "#4FAE7C", "warn": "#DDA43A", "offline": "#D9564F"}

BG = "#14171A"
PANEL = "#1B1F23"
PANEL_ALT = "#20242A"
BORDER = "#2A2F34"
TEXT = "#E7E5E0"
MUTED = "#8B9199"
FAINT = "#5B6066"

AUTOSAVE_DELAY_MS = 600


def default_servo(servo_id, joint):
    return {
        "id": servo_id,
        "joint": joint,
        "status": "ok",
        "torque_limit": 70,
        "note": "",
        "updated": None,
    }


def default_arms():
    return {
        "right": {
            "label": "Right arm",
            "port": "/dev/dxl_right",
            "supply": "SMPS2Dynamixel — 12V, 5A",
            "note": "",
            "servos": [
                default_servo(1, "right_joint1"),
                default_servo(2, "right_joint2"),
                default_servo(3, "right_joint3"),
                default_servo(4, "right_joint4"),
                default_servo(5, "right_joint5"),
                default_servo(6, "right_gripper"),
            ],
        },
        "left": {
            "label": "Left arm",
            "port": "/dev/dxl_left",
            "supply": "SMPS2Dynamixel — 12V, 5A",
            "note": "",
            "servos": [
                default_servo(11, "left_joint1"),
                default_servo(12, "left_joint2"),
                default_servo(13, "left_joint3"),
                default_servo(14, "left_joint4"),
                default_servo(15, "left_joint5"),
                default_servo(16, "left_gripper"),
            ],
        },
    }


def load_arms():
    """Load persisted data, falling back to defaults on any problem
    (missing file, corrupt JSON, unexpected shape)."""

    if not os.path.exists(STORAGE_PATH):
        return default_arms()

    try:
        with open(STORAGE_PATH, "r") as f:
            data = json.load(f)
        if "right" in data and "left" in data:
            return data
    except (json.JSONDecodeError, OSError, KeyError):
        pass

    return default_arms()


def save_arms(arms):
    """Write to a temp file then rename, so a crash mid-write can't
    corrupt the existing log."""

    tmp_path = STORAGE_PATH + ".tmp"
    try:
        with open(tmp_path, "w") as f:
            json.dump(arms, f, indent=2)
        os.replace(tmp_path, STORAGE_PATH)
        return True
    except OSError:
        return False


def format_timestamp(iso_str):
    if not iso_str:
        return "never logged"
    try:
        dt = datetime.fromisoformat(iso_str)
        return dt.strftime("%b %-d, %-I:%M %p")
    except ValueError:
        return "never logged"


class ServoRow(ttk.Frame):
    """One servo's editable row: status, torque limit, note, timestamp."""

    def __init__(self, parent, servo, on_change, **kwargs):
        super().__init__(parent, style="Panel.TFrame", **kwargs)
        self.servo = servo
        self.on_change = on_change

        self.columnconfigure(3, weight=1)

        self.status_var = tk.StringVar(value=servo["status"])
        self.status_dot = tk.Label(
            self, text="●", fg=STATUS_COLORS[servo["status"]], bg=PANEL, font=("Helvetica", 11)
        )
        self.status_dot.grid(row=0, column=0, padx=(0, 4), pady=6, sticky="w")

        id_label = tk.Label(
            self, text=f"ID {servo['id']}", fg=TEXT, bg=PANEL, font=("Courier New", 10, "bold")
        )
        id_label.grid(row=0, column=1, padx=(0, 10), pady=6, sticky="w")

        self.status_menu = ttk.Combobox(
            self, textvariable=self.status_var, values=STATUS_OPTIONS,
            state="readonly", width=9, font=("Helvetica", 9),
        )
        self.status_menu.grid(row=0, column=2, padx=(0, 10), pady=6)
        self.status_menu.bind("<<ComboboxSelected>>", self._on_status_change)

        joint_label = tk.Label(
            self, text=servo["joint"], fg=MUTED, bg=PANEL, font=("Helvetica", 9), width=14, anchor="w"
        )
        joint_label.grid(row=0, column=3, padx=(0, 10), pady=6, sticky="w")

        torque_frame = tk.Frame(self, bg=PANEL)
        torque_frame.grid(row=0, column=4, padx=(0, 10), pady=6)
        self.torque_var = tk.StringVar(value=str(servo["torque_limit"]))
        torque_entry = tk.Entry(
            torque_frame, textvariable=self.torque_var, width=4,
            bg=PANEL_ALT, fg=TEXT, insertbackground=TEXT,
            relief="flat", highlightthickness=1, highlightbackground=BORDER,
            font=("Courier New", 9),
        )
        torque_entry.pack(side="left")
        torque_entry.bind("<FocusOut>", self._on_torque_change)
        torque_entry.bind("<Return>", self._on_torque_change)
        tk.Label(torque_frame, text="% torque", fg=FAINT, bg=PANEL, font=("Helvetica", 8)).pack(
            side="left", padx=(4, 0)
        )

        self.note_var = tk.StringVar(value=servo["note"])
        note_entry = tk.Entry(
            self, textvariable=self.note_var,
            bg=PANEL, fg=TEXT, insertbackground=TEXT,
            relief="flat", highlightthickness=0, font=("Helvetica", 9),
        )
        note_entry.grid(row=0, column=5, padx=(0, 10), pady=6, sticky="ew")
        note_entry.bind("<KeyRelease>", self._on_note_change)
        note_entry.insert(0, "") if not servo["note"] else None
        if not servo["note"]:
            note_entry.insert(0, "")

        self.timestamp_label = tk.Label(
            self, text=format_timestamp(servo["updated"]), fg=FAINT, bg=PANEL, font=("Helvetica", 8)
        )
        self.timestamp_label.grid(row=0, column=6, pady=6, sticky="e")

    def _touch(self):
        self.servo["updated"] = datetime.now().isoformat()
        self.timestamp_label.config(text=format_timestamp(self.servo["updated"]))
        self.on_change()

    def _on_status_change(self, _event):
        new_status = self.status_var.get()
        self.servo["status"] = new_status
        self.status_dot.config(fg=STATUS_COLORS[new_status])
        self._touch()

    def _on_torque_change(self, _event):
        raw = self.torque_var.get().strip()
        try:
            value = max(0, min(100, int(raw)))
        except ValueError:
            value = self.servo["torque_limit"]
        self.torque_var.set(str(value))
        if value != self.servo["torque_limit"]:
            self.servo["torque_limit"] = value
            self._touch()

    def _on_note_change(self, _event):
        new_note = self.note_var.get()
        if new_note != self.servo["note"]:
            self.servo["note"] = new_note
            self._touch()


class ArmPanel(ttk.Frame):
    """One arm's panel: header (port/supply/notes) plus its 6 servo rows."""

    def __init__(self, parent, arm_data, on_change, **kwargs):
        super().__init__(parent, style="Panel.TFrame", **kwargs)
        self.arm_data = arm_data
        self.on_change = on_change

        header = tk.Frame(self, bg=PANEL)
        header.pack(fill="x", padx=12, pady=(10, 6))

        tk.Label(
            header, text=arm_data["label"], fg=TEXT, bg=PANEL, font=("Helvetica", 12, "bold")
        ).pack(anchor="w")

        self.port_var = tk.StringVar(value=arm_data["port"])
        port_entry = tk.Entry(
            header, textvariable=self.port_var, bg=PANEL, fg=MUTED,
            relief="flat", highlightthickness=0, font=("Courier New", 9),
        )
        port_entry.pack(anchor="w", fill="x", pady=(4, 0))
        port_entry.bind("<KeyRelease>", lambda e: self._on_field_change("port", self.port_var.get()))

        self.supply_var = tk.StringVar(value=arm_data["supply"])
        supply_entry = tk.Entry(
            header, textvariable=self.supply_var, bg=PANEL, fg=MUTED,
            relief="flat", highlightthickness=0, font=("Helvetica", 9),
        )
        supply_entry.pack(anchor="w", fill="x", pady=(2, 0))
        supply_entry.bind(
            "<KeyRelease>", lambda e: self._on_field_change("supply", self.supply_var.get())
        )

        rows_frame = tk.Frame(self, bg=PANEL, highlightbackground=BORDER, highlightthickness=1)
        rows_frame.pack(fill="x", padx=12)
        for servo in arm_data["servos"]:
            row = ServoRow(rows_frame, servo, on_change)
            row.pack(fill="x")

        note_frame = tk.Frame(self, bg=PANEL)
        note_frame.pack(fill="x", padx=12, pady=(8, 12))
        tk.Label(
            note_frame, text="Arm notes", fg=FAINT, bg=PANEL, font=("Helvetica", 8)
        ).pack(anchor="w")
        self.arm_note_var = tk.StringVar(value=arm_data["note"])
        arm_note_entry = tk.Entry(
            note_frame, textvariable=self.arm_note_var, bg=PANEL, fg=TEXT,
            insertbackground=TEXT, relief="flat", highlightthickness=1,
            highlightbackground=BORDER, font=("Helvetica", 9),
        )
        arm_note_entry.pack(fill="x", pady=(3, 0))
        arm_note_entry.bind(
            "<KeyRelease>", lambda e: self._on_field_change("note", self.arm_note_var.get())
        )

    def _on_field_change(self, field, value):
        self.arm_data[field] = value
        self.on_change()


class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Arm servo log")
        self.geometry("980x640")
        self.configure(bg=BG)

        style = ttk.Style(self)
        style.theme_use("clam")
        style.configure("Panel.TFrame", background=PANEL)
        style.configure(
            "TCombobox", fieldbackground=PANEL_ALT, background=PANEL_ALT,
            foreground=TEXT, arrowcolor=TEXT,
        )

        self.arms = load_arms()
        self._save_job = None

        self._build_header()

        content = tk.Frame(self, bg=BG)
        content.pack(fill="both", expand=True, padx=16, pady=(4, 16))
        content.columnconfigure(0, weight=1)
        content.columnconfigure(1, weight=1)

        self.right_panel = ArmPanel(content, self.arms["right"], self._schedule_save)
        self.right_panel.grid(row=0, column=0, sticky="nsew", padx=(0, 8))

        self.left_panel = ArmPanel(content, self.arms["left"], self._schedule_save)
        self.left_panel.grid(row=0, column=1, sticky="nsew", padx=(8, 0))

        self.protocol("WM_DELETE_WINDOW", self._on_close)

    def _build_header(self):
        header = tk.Frame(self, bg=BG)
        header.pack(fill="x", padx=16, pady=(16, 4))

        left = tk.Frame(header, bg=BG)
        left.pack(side="left")
        tk.Label(
            left, text="Arm servo log", fg=TEXT, bg=BG, font=("Helvetica", 15, "bold")
        ).pack(anchor="w")
        tk.Label(
            left, text="Per-servo status, torque limits, and field notes for both arms.",
            fg=MUTED, bg=BG, font=("Helvetica", 9),
        ).pack(anchor="w")

        right = tk.Frame(header, bg=BG)
        right.pack(side="right")
        self.save_status_label = tk.Label(
            right, text="Saved", fg=MUTED, bg=BG, font=("Helvetica", 9)
        )
        self.save_status_label.pack(side="left", padx=(0, 10))

        reset_btn = tk.Button(
            right, text="Reset log", command=self._on_reset,
            bg=BG, fg=MUTED, activebackground=PANEL_ALT, activeforeground=TEXT,
            relief="flat", highlightthickness=1, highlightbackground=BORDER,
            font=("Helvetica", 9), padx=8, pady=3, bd=0,
        )
        reset_btn.pack(side="left")

    def _schedule_save(self):
        self.save_status_label.config(text="Saving…", fg=MUTED)
        if self._save_job is not None:
            self.after_cancel(self._save_job)
        self._save_job = self.after(AUTOSAVE_DELAY_MS, self._save_now)

    def _save_now(self):
        ok = save_arms(self.arms)
        if ok:
            self.save_status_label.config(text="Saved", fg=MUTED)
        else:
            self.save_status_label.config(text="Save failed", fg=STATUS_COLORS["offline"])
        self._save_job = None

    def _on_reset(self):
        self.arms = default_arms()
        save_arms(self.arms)
        # Simplest reliable way to reflect the reset in the UI: rebuild.
        for widget in self.winfo_children():
            widget.destroy()
        self._save_job = None
        self._build_header()
        content = tk.Frame(self, bg=BG)
        content.pack(fill="both", expand=True, padx=16, pady=(4, 16))
        content.columnconfigure(0, weight=1)
        content.columnconfigure(1, weight=1)
        self.right_panel = ArmPanel(content, self.arms["right"], self._schedule_save)
        self.right_panel.grid(row=0, column=0, sticky="nsew", padx=(0, 8))
        self.left_panel = ArmPanel(content, self.arms["left"], self._schedule_save)
        self.left_panel.grid(row=0, column=1, sticky="nsew", padx=(8, 0))

    def _on_close(self):
        if self._save_job is not None:
            self.after_cancel(self._save_job)
        save_arms(self.arms)
        self.destroy()


if __name__ == "__main__":
    app = App()
    app.mainloop()