"""
ROS-free core of nova_arm_tactile (taken from FlexiGrasp): wire-format parser, baseline, features,
serial channel, demo source.

Wire format (unchanged from the FlexiTac Arduino firmware): 0xAA 0x55 followed by 512 bytes
(16 rows x 32 cols, row-major, 8-bit ADC). A frame is only accepted if the *next* header sits
exactly 512 bytes after the payload, so a single lost or added byte rejects that frame instead of
shifting every taxel by one.

Changes from FlexiGrasp: an optional active region (taxels outside it are ignored like masked
ones), connect-attempt and frame counters for the nova_arm health convention, and a demo stream
whose press can be driven from outside (the simulated gripper position).

Nothing here imports rclpy, so it can be tested and reused on its own.
"""
from __future__ import annotations

import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Callable, List, Optional, Tuple

import numpy as np

ROWS, COLS = 16, 32
N_TAXELS = ROWS * COLS
FRAME_BYTES = N_TAXELS
MAGIC = b"\xAA\x55"
DEFAULT_BAUD = 2_000_000

_RR, _CC = np.indices((ROWS, COLS)).astype(np.float32)


# ----------------------------------------------------------------------------- parsing
class FrameParser:
    """Incremental parser: feed(bytes) returns the list of complete, validated frames."""

    MAX_RING = 50_000

    def __init__(self) -> None:
        self.ring = bytearray()
        self.synced = False
        self.bad_frames = 0
        self.dropped_bytes = 0
        self.sync_errors = 0

    def _drop(self, n: int) -> None:
        if self.synced and n > 0:
            self.sync_errors += 1
            self.dropped_bytes += n

    def feed(self, chunk: bytes) -> List[np.ndarray]:
        ring = self.ring
        ring.extend(chunk)
        if len(ring) > self.MAX_RING:
            excess = len(ring) - self.MAX_RING
            self._drop(excess)
            del ring[:excess]
        frames: List[np.ndarray] = []
        while True:
            idx = ring.find(MAGIC)
            if idx < 0:
                if len(ring) > 1:
                    self._drop(len(ring) - 1)
                    del ring[: len(ring) - 1]  # keep the last byte, the marker may be split
                break
            if idx > 0:
                self._drop(idx)
                del ring[:idx]
            if len(ring) < 2 + FRAME_BYTES + 2:
                break  # need the payload plus the next header to validate
            if bytes(ring[2 + FRAME_BYTES:4 + FRAME_BYTES]) != MAGIC:
                self._drop(1)
                self.bad_frames += 1
                del ring[:1]
                continue
            payload = bytes(ring[2:2 + FRAME_BYTES])
            del ring[:2 + FRAME_BYTES]
            frames.append(np.frombuffer(payload, dtype=np.uint8).reshape(ROWS, COLS).copy())
            self.synced = True
        return frames


# ----------------------------------------------------------------------------- features
def parse_mask(text: str) -> np.ndarray:
    """'0,10;0,11;1,30' -> boolean (ROWS, COLS) array, True where a taxel is masked."""
    mask = np.zeros((ROWS, COLS), dtype=bool)
    for item in text.replace(" ", "").split(";"):
        if not item:
            continue
        r, c = item.split(",")
        r, c = int(r), int(c)
        if not (0 <= r < ROWS and 0 <= c < COLS):
            raise ValueError(f"masked taxel ({r},{c}) is outside the {ROWS}x{COLS} pad")
        mask[r, c] = True
    return mask


def parse_region(text: str) -> np.ndarray:
    """
    'r0:r1,c0:c1' (half-open, like numpy slices) -> boolean (ROWS, COLS) array, True OUTSIDE the
    region, so it can be OR-ed into the mask. Empty text means the whole pad is active.
    Example: '2:14,0:32' ignores the two outer rows on each long edge.
    """
    outside = np.zeros((ROWS, COLS), dtype=bool)
    text = text.replace(" ", "")
    if not text:
        return outside
    try:
        rs, cs = text.split(",")
        r0, r1 = (int(v) for v in rs.split(":"))
        c0, c1 = (int(v) for v in cs.split(":"))
    except ValueError as exc:
        raise ValueError(f"active region '{text}' is not 'r0:r1,c0:c1'") from exc
    if not (0 <= r0 < r1 <= ROWS and 0 <= c0 < c1 <= COLS):
        raise ValueError(f"active region '{text}' is empty or outside the {ROWS}x{COLS} pad")
    outside[:, :] = True
    outside[r0:r1, c0:c1] = False
    return outside


@dataclass
class Features:
    delta: np.ndarray
    peak: float
    total: float
    contact_taxels: int
    in_contact: bool
    centroid_row: float
    centroid_col: float


def compute_features(raw_f32: np.ndarray, baseline: np.ndarray, mask: np.ndarray,
                     threshold: float, min_taxels: int) -> Features:
    delta = raw_f32 - baseline
    delta[mask] = 0.0
    pos = np.clip(delta - threshold, 0.0, None)
    total = float(pos.sum())
    n = int((delta > threshold).sum())
    if total > 0.0:
        cr = float((pos * _RR).sum() / total)
        cc = float((pos * _CC).sum() / total)
    else:
        cr = cc = float("nan")
    return Features(delta, float(delta.max()), total, n, n >= min_taxels, cr, cc)


@dataclass
class ProcessedFrame:
    stamp_ns: int
    seq: int
    raw: np.ndarray            # uint8 (ROWS, COLS)
    calibrated: bool
    feat: Features
    bad_frames: int
    dropped_bytes: int
    rate_hz: float


# ----------------------------------------------------------------------------- serial channel
class FingerChannel(threading.Thread):
    """
    Owns one pad: opens the port (retrying if it is missing), parses frames, keeps the baseline,
    and queues ProcessedFrame objects for the ROS node to publish.
    """

    def __init__(self, name: str, port: str, baud: int = DEFAULT_BAUD, baseline_frames: int = 30,
                 threshold: float = 15.0, min_taxels: int = 2, mask: Optional[np.ndarray] = None,
                 opener: Optional[Callable[[], object]] = None, log: Callable[[str], None] = print,
                 queue_len: int = 1000) -> None:
        super().__init__(daemon=True, name=f"finger-{name}")
        self.name_ = name
        self.port = port
        self.baud = baud
        self.baseline_frames = max(5, int(baseline_frames))
        self.threshold = float(threshold)
        self.min_taxels = int(min_taxels)
        self.mask = mask if mask is not None else np.zeros((ROWS, COLS), dtype=bool)
        self._opener = opener
        self._log = log
        self.max_backoff = 5.0            # s between reopen attempts once backed off
        self.queue: deque = deque(maxlen=queue_len)
        self.lock = threading.Lock()
        self._stop_evt = threading.Event()

        self.connected = False
        self.last_error = ""
        self.connect_attempts = 0
        self.parser = FrameParser()
        self.seq = 0
        self.baseline: Optional[np.ndarray] = None
        self.noise_std: Optional[np.ndarray] = None
        self._calib: List[np.ndarray] = []
        self._calibrating = True
        self.baseline_note = ""
        self._times: deque = deque(maxlen=1024)
        self.last_frame_time = 0.0

    # -- control
    def stop(self) -> None:
        self._stop_evt.set()

    def rezero(self) -> str:
        with self.lock:
            self._calib = []
            self._calibrating = True
        return f"{self.name_}: collecting {self.baseline_frames} frames for a new baseline (keep the pad unloaded)"

    @property
    def calibrating(self) -> bool:
        return self._calibrating

    def info(self) -> dict:
        """Snapshot for the health status (same spirit as nova_arm_driver's ArmBus.info())."""
        with self.lock:
            noisy = (int((self.noise_std[~self.mask] > 2.0).sum())
                     if self.noise_std is not None else -1)
            return {
                "connected": self.connected,
                "port": self.port,
                "connect_attempts": self.connect_attempts,
                "last_error": self.last_error,
                "frames": self.seq,
                "last_frame_time": self.last_frame_time,
                "rate_hz": float(len(self._times)),
                "calibrated": self.baseline is not None and not self._calibrating,
                "calibrating": self._calibrating,
                "bad_frames": self.parser.bad_frames,
                "dropped_bytes": self.parser.dropped_bytes,
                "noisy_taxels": noisy,
                "baseline_note": self.baseline_note,
            }

    # -- internals
    def _open(self):
        if self._opener is not None:
            return self._opener()
        import serial  # pyserial, imported lazily so demo/tests do not need it
        return serial.Serial(self.port, self.baud, timeout=0.02)

    def run(self) -> None:
        backoff = min(0.5, self.max_backoff)
        while not self._stop_evt.is_set():
            self.connect_attempts += 1
            try:
                ser = self._open()
            except Exception as exc:  # noqa: BLE001 - any open failure means "retry"
                msg = f"{type(exc).__name__}: {exc}"
                if msg != self.last_error:
                    self._log(f"[{self.name_}] cannot open {self.port}: {msg} (retrying)")
                self.last_error = msg
                self.connected = False
                self._stop_evt.wait(backoff)
                backoff = min(backoff * 2, self.max_backoff)
                continue
            backoff = min(0.5, self.max_backoff)
            self.connected = True
            self.last_error = ""
            self.parser = FrameParser()
            self._log(f"[{self.name_}] opened {self.port}")
            try:
                while not self._stop_evt.is_set():
                    chunk = ser.read(8192)
                    if not chunk:
                        continue
                    t_ns = time.time_ns()
                    for f in self.parser.feed(chunk):
                        self._on_frame(f, t_ns)
            except Exception as exc:  # noqa: BLE001 - unplugged, EIO, etc.
                self.last_error = f"{type(exc).__name__}: {exc}"
                self._log(f"[{self.name_}] link lost: {self.last_error} (reopening)")
            finally:
                self.connected = False
                try:
                    ser.close()
                except Exception:  # noqa: BLE001
                    pass

    def _on_frame(self, frame_u8: np.ndarray, t_ns: int) -> None:
        f = frame_u8.astype(np.float32)
        now = time.time()
        with self.lock:
            self.seq += 1
            self._times.append(now)
            self.last_frame_time = now
            while self._times and self._times[0] < now - 1.0:
                self._times.popleft()
            if self._calibrating:
                self._calib.append(f)
                if len(self._calib) >= self.baseline_frames:
                    stack = np.stack(self._calib)
                    self.baseline = np.median(stack, axis=0)
                    self.noise_std = stack.std(axis=0)
                    self._calibrating = False
                    self._calib = []
                    stuck = int((self.baseline[~self.mask] >= 100).sum())
                    noisy = int((self.noise_std[~self.mask] > 2.0).sum())
                    self.baseline_note = (f"baseline set: median raw {np.median(self.baseline):.1f}, "
                                          f"{stuck} taxels >=100 (touching during zeroing?), "
                                          f"{noisy} taxels with sigma>2")
                    self._log(f"[{self.name_}] {self.baseline_note}")
            base = self.baseline
            calibrated = base is not None and not self._calibrating
            if base is None:
                base = f  # not used for features while uncalibrated
            rate = float(len(self._times))
            seq = self.seq
        if calibrated:
            feat = compute_features(f, base, self.mask, self.threshold, self.min_taxels)
        else:
            feat = Features(np.zeros((ROWS, COLS), np.float32), 0.0, 0.0, 0, False,
                            float("nan"), float("nan"))
        self.queue.append(ProcessedFrame(t_ns, seq, frame_u8, calibrated, feat,
                                         self.parser.bad_frames, self.parser.dropped_bytes, rate))

    def drain(self) -> List[ProcessedFrame]:
        out = []
        while True:
            try:
                out.append(self.queue.popleft())
            except IndexError:
                return out


# ----------------------------------------------------------------------------- demo source
class DemoStream:
    """
    Stands in for serial.Serial: emits the real wire format.

    Without `press`, a press wanders over the pad on its own (as in FlexiGrasp). With `press`, the
    callable is asked every frame for (amplitude, centre_row, centre_col); the node uses that to
    turn the simulated gripper position into pad pressure so the grasp controller can be exercised
    without hardware.
    """

    def __init__(self, rate_hz: float = 98.0, glitch: bool = False, seed: int = 0,
                 phase: float = 0.0,
                 press: Optional[Callable[[], Tuple[float, float, float]]] = None) -> None:
        self.rng = np.random.default_rng(seed)
        self.dt = 1.0 / rate_hz
        self.glitch = glitch
        self.phase = phase
        self.press = press
        self.t0 = self.next = time.time()
        self.base = 2.0 + self.rng.uniform(0.0, 2.0, (ROWS, COLS))
        self.closed = False

    def read(self, n: int) -> bytes:
        now = time.time()
        if self.next < now - 1.0:
            self.next = now
        wait = self.next - now
        if wait > 0:
            time.sleep(min(wait, 0.05))
            if self.next - time.time() > 0:
                return b""
        self.next += self.dt
        t = time.time() - self.t0
        if self.press is not None:
            amp, cr, cc = self.press()
            sigma = 2.5
        else:
            ph = (t - 2.0 + self.phase) % 8.0
            amp = 90.0 * np.sin(np.pi * ph / 4.0) ** 2 if (t > 2.0 and ph < 4.0) else 0.0
            cr = 8 + 4 * np.sin(2 * np.pi * t / 13.0 + self.phase)
            cc = 16 + 9 * np.sin(2 * np.pi * t / 7.0 + self.phase)
            sigma = 1.8
        blob = amp * np.exp(-((_RR - cr) ** 2 + (_CC - cc) ** 2) / (2 * sigma ** 2))
        frame = self.base + blob + self.rng.normal(0.0, 0.8, (ROWS, COLS))
        frame = np.clip(np.rint(frame), 0, 255).astype(np.uint8)
        data = MAGIC + frame.tobytes()
        if self.glitch and self.rng.random() < 0.02:
            k = int(self.rng.integers(2, len(data)))
            data = data[:k] + data[k + 1:]
        return data

    def close(self) -> None:
        self.closed = True