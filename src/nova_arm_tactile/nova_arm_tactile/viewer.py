"""
Live pad viewer (matplotlib), from FlexiGrasp. ROS-free: it is fed ViewFrame objects and calls
back for re-zero, so it can be rendered headless in tests. gui.py wires it to ROS topics.
For nova_arm each panel is one arm's gripper pad; the left/right ratio is off by default because
the two pads are on different grippers.
"""
from __future__ import annotations

import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Callable, Dict, List, Optional

import numpy as np

ROWS, COLS = 16, 32

SURFACE, INK, INK2, MUTED = "#fcfcfb", "#0b0b0b", "#52514e", "#898781"
GRID, AXIS = "#e1e0d9", "#c3c2b7"
FINGER_COLORS = ["#2a78d6", "#eb6834", "#2ba58a", "#8a63d2"]
SEQ_BLUE = ["#cde2fb", "#9ec5f4", "#5598e7", "#2a78d6", "#1c5cab", "#104281", "#0d366b"]
DIV = ["#256abf", "#f0efec", "#e34948"]
STATUS = {"good": ("#0ca30c", "#ffffff"), "warning": ("#fab219", "#0b0b0b"),
          "critical": ("#d03b3b", "#ffffff"), "info": ("#e1e0d9", "#0b0b0b")}


@dataclass
class ViewFrame:
    t: float                  # host time in seconds
    seq: int
    raw: np.ndarray           # float32 (ROWS, COLS)
    delta: np.ndarray         # float32 (ROWS, COLS)
    calibrated: bool
    peak: float
    total: float
    contact_taxels: int
    in_contact: bool
    centroid_row: float
    centroid_col: float
    bad_frames: int
    dropped_bytes: int
    rate_hz: float


class FingerView:
    """Latest frame and a short history for one finger."""

    def __init__(self, name: str, history: int = 4000) -> None:
        self.name = name
        self.lock = threading.Lock()
        self.latest: Optional[ViewFrame] = None
        self.hist: deque = deque(maxlen=history)   # (t, total, peak)
        self.first_bad: Optional[int] = None
        self.bad_at_first_seen = 0

    def push(self, f: ViewFrame) -> None:
        with self.lock:
            if self.latest is None:
                self.bad_at_first_seen = f.bad_frames
            self.latest = f
            self.hist.append((f.t, f.total, f.peak))

    def snapshot(self):
        with self.lock:
            return self.latest, list(self.hist)


class DualViewer:
    def __init__(self, names: List[str], rezero: Callable[[Optional[str]], None],
                 window_s: float = 15.0, delta_range: float = 120.0, threshold: float = 15.0,
                 stale_s: float = 1.0, title: str = "nova_arm gripper pads",
                 show_balance: bool = False) -> None:
        self.title = title
        self.show_balance = show_balance
        self.names = names
        self.views: Dict[str, FingerView] = {n: FingerView(n) for n in names}
        self.rezero = rezero
        self.window_s = window_s
        self.delta_range = delta_range
        self.threshold = threshold
        self.stale_s = stale_s
        self.paused = False
        self.show_raw = False
        self.subtitle_fn: Optional[Callable[[], str]] = None   # extra status line (grasp state)
        self._build()

    # ------------------------------------------------------------------ figure
    def _build(self) -> None:
        import matplotlib.pyplot as plt
        from matplotlib.colors import LinearSegmentedColormap
        from matplotlib.gridspec import GridSpec
        from matplotlib.widgets import Button

        self.plt = plt
        plt.rcParams.update({
            "toolbar": "None", "font.size": 9, "text.color": INK,
            "axes.edgecolor": AXIS, "axes.labelcolor": INK2,
            "xtick.color": MUTED, "ytick.color": MUTED,
            "figure.facecolor": SURFACE, "axes.facecolor": SURFACE, "savefig.facecolor": SURFACE})
        for k in ("keymap.home", "keymap.pan", "keymap.zoom", "keymap.grid", "keymap.grid_minor",
                  "keymap.xscale", "keymap.yscale", "keymap.back", "keymap.forward", "keymap.save",
                  "keymap.fullscreen", "keymap.quit"):
            plt.rcParams[k] = []
        self.cmap_div = LinearSegmentedColormap.from_list("div", DIV)
        self.cmap_seq = LinearSegmentedColormap.from_list("seq", SEQ_BLUE)

        n = len(self.names)
        self.fig = fig = plt.figure(figsize=(6.2 * n, 8.4))
        gs = GridSpec(3, n, figure=fig, left=0.05, right=0.96, top=0.87, bottom=0.11,
                      hspace=0.42, wspace=0.18, height_ratios=[1.0, 0.95, 0.72])
        fig.text(0.05, 0.955, self.title, fontsize=14, fontweight="bold", color=INK)
        self.subtitle = fig.text(0.05, 0.925, "", fontsize=9, color=INK2)
        fig.text(0.05, 0.02, "space pause   |   r re-zero all   |   v raw / delta   |   +/- delta range   |   q quit",
                 fontsize=8.5, color=MUTED)

        zeros = np.zeros((ROWS, COLS), dtype=np.float32)
        self.ims, self.cen, self.heat_titles, self.banners, self.stats = {}, {}, {}, {}, {}
        for i, name in enumerate(self.names):
            ax = fig.add_subplot(gs[0, i])
            for s in ("top", "right"):
                ax.spines[s].set_visible(False)
            self.heat_titles[name] = ax.set_title("", loc="left", fontsize=10, fontweight="bold", color=INK)
            im = ax.imshow(zeros, cmap=self.cmap_div, vmin=-self.delta_range, vmax=self.delta_range,
                           interpolation="nearest", aspect="equal")
            ax.set_xticks([0, 7, 15, 23, 31]); ax.set_yticks([0, 5, 10, 15])
            ax.set_xlabel("column"); ax.set_ylabel("row")
            fig.colorbar(im, ax=ax, fraction=0.035, pad=0.02)
            (cm,) = ax.plot([], [], ls="", marker="x", ms=10, mew=2.2, color=INK)
            self.ims[name], self.cen[name] = im, cm

            axs = fig.add_subplot(gs[2, i])
            axs.axis("off")
            self.banners[name] = axs.text(0.0, 0.99, "", transform=axs.transAxes, va="top", ha="left",
                                          fontsize=9, fontweight="bold")
            self.stats[name] = axs.text(0.0, 0.70, "", transform=axs.transAxes, va="top", ha="left",
                                        family="monospace", fontsize=8.4, color=INK)

        self.ax_ts = fig.add_subplot(gs[1, :])
        ax = self.ax_ts
        for s in ("top", "right"):
            ax.spines[s].set_visible(False)
        ax.set_title("Total contact signal  (sum of delta above threshold)", loc="left",
                     fontsize=10, fontweight="bold", color=INK)
        ax.set_xlabel("seconds ago"); ax.set_ylabel("counts")
        ax.grid(True, color=GRID, lw=0.6); ax.set_axisbelow(True)
        ax.set_xlim(-self.window_s, 0); ax.set_ylim(0, 300)
        self.lines = {}
        for i, name in enumerate(self.names):
            (ln,) = ax.plot([], [], color=FINGER_COLORS[i % len(FINGER_COLORS)], lw=2, label=name)
            self.lines[name] = ln
        ax.legend(loc="upper left", frameon=False, fontsize=9, ncol=n)
        self.balance = ax.text(0.995, 0.97, "", transform=ax.transAxes, ha="right", va="top",
                               fontsize=9, color=INK2, family="monospace")

        # buttons
        self.buttons = []
        specs = [(f"Re-zero {nm}", (lambda _e, nm=nm: self.rezero(nm))) for nm in self.names]
        specs.append(("Re-zero all", lambda _e: self.rezero(None)))
        specs.append(("Pause", self._toggle_pause))
        w = 0.12
        for j, (label, cb) in enumerate(reversed(specs)):
            axb = fig.add_axes([0.985 - (j + 1) * (w + 0.008), 0.012, w, 0.045])
            b = Button(axb, label, color="#e1e0d9", hovercolor="#cde2fb")
            b.label.set_fontsize(9); b.label.set_color(INK)
            b.on_clicked(cb)
            self.buttons.append(b)
            if label == "Pause":
                self.pause_button = b
        fig.canvas.mpl_connect("key_press_event", self._on_key)

    # ------------------------------------------------------------------ interaction
    def _toggle_pause(self, _e=None) -> None:
        self.paused = not self.paused
        self.pause_button.label.set_text("Resume" if self.paused else "Pause")

    def _on_key(self, ev) -> None:
        k = ev.key
        if k == " ":
            self._toggle_pause()
        elif k == "r":
            self.rezero(None)
        elif k == "v":
            self.show_raw = not self.show_raw
        elif k in ("+", "="):
            self.delta_range = min(self.delta_range * 1.25, 255.0)
        elif k == "-":
            self.delta_range = max(self.delta_range / 1.25, 10.0)
        elif k == "q":
            self.plt.close(self.fig)

    # ------------------------------------------------------------------ drawing
    def update(self, _frame=None) -> None:
        if self.paused:
            return
        now = time.time()
        totals = {}
        ymax = 50.0
        for name in self.names:
            f, hist = self.views[name].snapshot()
            im, title = self.ims[name], self.heat_titles[name]
            if f is None:
                title.set_text(f"{name} gripper pad")
                self._banner(name, "critical", "NO DATA: nothing received yet.\nCheck the node, port, power, baud.")
                self.stats[name].set_text("")
                continue
            age = now - f.t
            data, cmap, vmin, vmax = ((f.raw, self.cmap_seq, 0, 255) if self.show_raw
                                      else (f.delta, self.cmap_div, -self.delta_range, self.delta_range))
            im.set_data(data); im.set_cmap(cmap); im.set_clim(vmin, vmax)
            title.set_text(f"{name} gripper pad  ({'raw ADC' if self.show_raw else 'delta = raw - baseline'})")
            if np.isfinite(f.centroid_row) and f.calibrated:
                self.cen[name].set_data([f.centroid_col], [f.centroid_row])
            else:
                self.cen[name].set_data([], [])

            if age > self.stale_s:
                self._banner(name, "critical", f"STALE: no frame for {age:.1f} s")
            elif not f.calibrated:
                self._banner(name, "info", "CALIBRATING: keep the pad unloaded")
            elif f.in_contact:
                self._banner(name, "good", f"CONTACT: {f.contact_taxels} taxels")
            elif f.bad_frames - self.views[name].bad_at_first_seen > 0 and f.rate_hz < 50:
                self._banner(name, "warning", "LINK: low rate and rejected frames")
            else:
                self._banner(name, "info", "no contact")
            cen = (f"({f.centroid_row:5.1f}, {f.centroid_col:5.1f})" if np.isfinite(f.centroid_row)
                   else "(  -  ,   -  )")
            self.stats[name].set_text(
                f"frame rate       {f.rate_hz:6.1f} Hz\n"
                f"frames           {f.seq:6d}\n"
                f"rejected frames  {f.bad_frames:6d}\n"
                f"dropped bytes    {f.dropped_bytes:6d}\n"
                f"peak delta       {f.peak:6.1f}\n"
                f"total            {f.total:6.1f}\n"
                f"contact taxels   {f.contact_taxels:6d}\n"
                f"centroid r,c   {cen}")
            sel = [(t, v) for t, v, _p in hist if t >= now - self.window_s]
            if sel:
                ts = np.array([t - now for t, _ in sel]); vs = np.array([v for _, v in sel])
                self.lines[name].set_data(ts, vs)
                ymax = max(ymax, float(vs.max()))
            totals[name] = f.total if (f.calibrated and age <= self.stale_s) else None
        self.ax_ts.set_ylim(0, ymax * 1.15)
        if self.show_balance and len(self.names) == 2 and all(totals.get(n) is not None for n in self.names):
            a, b = (totals[n] for n in self.names)
            self.balance.set_text(f"{self.names[0]}/{self.names[1]} = {a / b:5.2f}" if b > 1e-6 and a > 1e-6
                                  else f"{self.names[0]}/{self.names[1]} =   -  ")
        else:
            self.balance.set_text("")
        extra = self.subtitle_fn() if self.subtitle_fn else ""
        self.subtitle.set_text(("live" if not self.paused else "paused") + (f"   |   {extra}" if extra else ""))

    def _banner(self, name: str, kind: str, text: str) -> None:
        bg, fg = STATUS[kind]
        b = self.banners[name]
        b.set_text(text); b.set_color(fg)
        b.set_bbox(dict(facecolor=bg, edgecolor="none", pad=5))

    def run(self, interval_ms: int = 40) -> None:
        from matplotlib.animation import FuncAnimation
        self.anim = FuncAnimation(self.fig, self.update, interval=interval_ms, cache_frame_data=False)
        self.plt.show()