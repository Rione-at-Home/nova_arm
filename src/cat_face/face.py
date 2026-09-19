#!/usr/bin/env python3
"""Robot face with a w mouth, neural TTS, happy idle, and two terminals.

SETUP (run in the folder containing this script; 64-bit Python recommended):
    python3 -m pip install pygame "piper-tts>=1.3,<2"
Download these two files once, then put them beside robot_cat.py:
    https://huggingface.co/rhasspy/piper-voices/resolve/v1.0.0/en/en_US/lessac/medium/en_US-lessac-medium.onnx
    https://huggingface.co/rhasspy/piper-voices/resolve/v1.0.0/en/en_US/lessac/medium/en_US-lessac-medium.onnx.json
Linux / Raspberry Pi OS also:
    sudo apt install python3-tk
The package installation and voice download are one-time setup. Both the
.onnx and .onnx.json files can instead be copied from another computer.
Runtime uses local synthesis with ONNX telemetry disabled: no cloud TTS, automatic downloader, or
online fallback. Speech text stays on this computer. The two terminals
communicate only over 127.0.0.1; HTTP proxies are explicitly disabled.
Optional basic voice: controls --tts system (Linux: install espeak-ng;
Windows: pip install pyttsx3; macOS: uses the built-in say command).

IF SPEECH GETS STUCK:
    python3 robot_cat.py doctor
This tests synthesis and playback without needing the face. Each stage is
printed. A 45-second watchdog stops the diagnostic if a driver hangs.
After updating this file, close both old processes and restart both terminals.

TERMINAL 1 (face only; no text prompt):
    python3 robot_cat.py
    python3 robot_cat.py display --fullscreen
TERMINAL 2 (all controls):
    python3 robot_cat.py controls
    Hello, I am your robot.   # ordinary text is spoken
    /happy                   # hold happy, with blinking and idle gaze
    /sleepy 3                # sleepy for 3 seconds, then happy
    /sleepy                  # also /curious, /surprised, /neutral
    /idle                    # return to happy idle
    /blink                   # blink now
    /stop                    # cancel speech/preparation and queued speech
    /voices                  # list available voice IDs
    /say happy               # explicitly speak any text
    /help                    # print controls
    /quit                    # close controls; face stays open and idle

Expressions and stop remain available during speech generation/playback.
Speech is prepared in a separate process so the control prompt stays responsive.
/stop cancels speech preparation and playback, and clears queued text.
Ctrl+C at the control prompt stops speech; /quit exits the control terminal.
Close the face window or press Escape to stop the display.
Face keys: 1-5 expressions, Space blink, F fullscreen, H help, Escape quit.
Use --show-help to show keyboard hints on the face (hidden by default).
All processes must use the same --port (default 8765). Control is local-only.
The older 'speech' and 'expressions' modes now alias the combined controls.

Speech and animation:
    Default: local Piper neural TTS, en_US-lessac-medium.
    Keep en_US-lessac-medium.onnx and en_US-lessac-medium.onnx.json next to
    robot_cat.py. To use another local model:
        python3 robot_cat.py controls --voice /path/to/voice.onnx
    /voices lists local model files; it never fetches an online catalogue.
    --rate 175 uses the model's natural pace; 150 is slower. This is relative
    speed for Piper, rather than a guaranteed words-per-minute measurement.
    --tts system uses the installed offline OS voice and rate is words/minute.

    Happy is the default expression. /idle and expired timed expressions
    restore happy. /neutral remains available as an explicit expression.
    Piper generates PCM WAV directly in memory; pygame plays it on the face.
    Mouth opening follows audio volume in 20 ms blocks, with smooth
    transitions. Silence and /stop close the mouth. Idle animation continues.

References:
    https://github.com/OHF-Voice/piper1-gpl
    https://github.com/espeak-ng/espeak-ng
    https://pyttsx3.readthedocs.io/en/latest/engine.html
    macOS: man say

"""

import argparse
import array
import base64
import io
import json
import os
import queue
import subprocess
import sys
import tempfile
import threading
import uuid
import wave
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.error import URLError, HTTPError
from urllib.request import Request, build_opener, ProxyHandler
import math
import multiprocessing
import random
import shutil
import time
import tkinter as tk


class RobotCat:
    EXPRESSIONS = ("neutral", "happy", "sleepy", "curious", "surprised")
    BG = "#050b13"
    CYAN = "#79f8e8"
    DIM = "#244d59"
    PINK = "#ff94b8"

    def __init__(self, root, width=800, height=480, fullscreen=False,
                 hide_help=False, expression="happy"):
        self.root = root
        root.title("Robot Cat")
        root.geometry(f"{width}x{height}")
        root.minsize(240, 144)
        self.canvas = tk.Canvas(root, bg=self.BG, highlightthickness=0)
        self.canvas.pack(fill="both", expand=True)
        self.expression = "happy"
        self.set_expression(expression)
        self.fullscreen = fullscreen
        self.show_help = not hide_help
        self.mouth_open = 0.0
        self.mouth_width = 1.0
        self.expression_until = None
        self.controller = None
        self.demo = False
        self.closed = False
        self.frame_job = None
        self.started = self.last_frame = time.monotonic()
        self.next_blink = self.started + random.uniform(1.5, 3.5)
        self.blink_start = -100.0
        self.next_gaze = self.started + 1.5
        self.next_expression = self.started + 4.0
        self.gaze = [0.0, 0.0]
        self.gaze_target = [0.0, 0.0]
        self.openness = 1.0
        self.scale, self.ox, self.oy = 1.0, 0.0, 0.0
        root.attributes("-fullscreen", fullscreen)
        self.canvas.configure(cursor="none" if fullscreen else "")
        root.bind("<KeyPress>", self._key)
        root.protocol("WM_DELETE_WINDOW", self.close)
        root.after_idle(self.canvas.focus_set)
        self._frame()

    def set_expression(self, expression, duration=None):
        """Set neutral, happy, sleepy, curious, or surprised."""
        if expression not in self.EXPRESSIONS:
            raise ValueError(f"Expression must be one of {self.EXPRESSIONS}")
        self.expression = expression
        self.expression_until = (time.monotonic() + duration
                                 if duration is not None else None)

    def blink(self):
        self.blink_start = time.monotonic()
        self.next_blink = self.blink_start + random.uniform(2.5, 5.5)

    def close(self):
        self.closed = True
        if self.frame_job is not None:
            self.root.after_cancel(self.frame_job)
            self.frame_job = None
        if self.controller is not None:
            self.controller.close()
        self.root.destroy()

    def _key(self, event):
        key = event.keysym.lower()
        if key == "escape":
            self.close()
        elif key in ("1", "2", "3", "4", "5"):
            self.demo = False
            self.set_expression(self.EXPRESSIONS[int(key) - 1])
        elif key == "space":
            self.blink()
        elif key == "a":
            self.demo = not self.demo
            self.next_expression = time.monotonic() + 4.0
        elif key == "h":
            self.show_help = not self.show_help
        elif key == "f":
            self.fullscreen = not self.fullscreen
            self.root.attributes("-fullscreen", self.fullscreen)
            self.canvas.configure(cursor="none" if self.fullscreen else "")

    def _coords(self, values):
        return [v * self.scale + (self.ox if i % 2 == 0 else self.oy)
                for i, v in enumerate(values)]

    def _line(self, points, color=None, width=5, smooth=True):
        self.canvas.create_line(*self._coords(points), fill=color or self.CYAN,
                                width=max(1, width * self.scale),
                                smooth=smooth, splinesteps=24,
                                capstyle=tk.ROUND, joinstyle=tk.ROUND)

    def _oval(self, box, color):
        self.canvas.create_oval(*self._coords(box), fill=color, outline="")

    def _polygon(self, points, color):
        self.canvas.create_polygon(*self._coords(points), fill=color, outline="")

    def _eye(self, cx, cy, opening, happy=False):
        if happy:
            arch = 27 * opening
            self._line([cx - 48, cy + 8, cx, cy + 8 - arch * 2,
                        cx + 48, cy + 8], width=12)
            return
        half_h = max(3, 55 * opening)
        self._oval([cx - 52, cy - half_h, cx + 52, cy + half_h], self.CYAN)
        if half_h > 10:
            px, py = cx + self.gaze[0], cy + self.gaze[1] * opening
            pupil_w = 16 if self.expression == "surprised" else 9
            pupil_h = half_h * 0.72
            self._oval([px - pupil_w, py - pupil_h,
                        px + pupil_w, py + pupil_h], self.BG)
            if half_h > 25:
                self._oval([px + 13, cy - 23, px + 22, cy - 14], "#eaffff")

    def _draw(self, t, opening):
        c = self.canvas
        c.delete("all")
        w, h = max(1, c.winfo_width()), max(1, c.winfo_height())
        self.scale = min(w / 800, h / 480)
        self.ox = (w - 800 * self.scale) / 2
        self.oy = (h - 480 * self.scale) / 2
        bob = 3 * math.sin(t * 1.8)
        self.oy += bob * self.scale

        left_open = opening
        right_open = opening * (0.65 if self.expression == "curious" else 1)
        self._eye(285, 223, left_open, self.expression == "happy")
        self._eye(515, 223, right_open, self.expression == "happy")
        if self.expression == "curious":
            self._line([466, 155, 510, 143, 552, 152], width=4)
        elif self.expression == "surprised":
            self._line([247, 145, 285, 130, 321, 145], width=4)
            self._line([479, 145, 515, 130, 553, 145], width=4)

        # Two joined curved lobes form a rounded w. The lower contour peels
        # downward from exactly the same curve, so it closes back into w.
        top, bottom = mouth_contours(self.mouth_open, self.mouth_width)
        if self.mouth_open > 0.015:
            self._polygon([v for point in top + bottom[::-1] for v in point], "#351e35")
            self._line([v for point in bottom for v in point], width=5, smooth=False)
        self._line([v for point in top for v in point], width=5, smooth=False)
        for side in (-1, 1):
            for row in (-1, 0, 1):
                self._line([400 + side * 160, 295 + row * 15,
                            400 + side * 243, 295 + row * 27], width=4)
            if self.expression == "happy":
                for stripe in range(3):
                    x = 400 + side * 150 + stripe * 10
                    self._line([x, 274, x - 4, 284], self.PINK, 3)

        if self.expression == "sleepy":
            x, y = self._coords([635, 192 - (t % 2) * 15])
            c.create_text(x, y, text="z", fill=self.DIM,
                          font=("Helvetica", max(8, int(23 * self.scale))))
        if self.show_help:
            # Draw help independently of the breathing offset.
            self.oy -= bob * self.scale
            for y, text in ((412, "1 NEUTRAL   2 HAPPY   3 SLEEPY   4 CURIOUS   5 SURPRISED"),
                            (439, "SPACE blink   F fullscreen   H hide   ESC quit")):
                c.create_text(*self._coords([400, y]), text=text, fill="#659297",
                              font=("Helvetica", max(5, int(12 * self.scale))))

    def _frame(self):
        if self.closed:
            return
        now = time.monotonic()
        dt = min(now - self.last_frame, 0.1)
        self.last_frame = now
        if self.controller is not None:
            self.controller.tick()
        if self.expression_until is not None and now >= self.expression_until:
            self.set_expression("happy")
        target_open, target_width = (self.controller.mouth_pose()
                                    if self.controller else (0.0, 1.0))
        mouth_blend = 1 - math.exp(-dt * 30)
        self.mouth_open += (target_open - self.mouth_open) * mouth_blend
        self.mouth_width += (target_width - self.mouth_width) * mouth_blend
        if now >= self.next_blink:
            self.blink()
        if now >= self.next_gaze:
            self.gaze_target = [random.uniform(-16, 16), random.uniform(-7, 7)]
            self.next_gaze = now + random.uniform(1.0, 3.0)
        if self.demo and now >= self.next_expression:
            i = (self.EXPRESSIONS.index(self.expression) + 1) % len(self.EXPRESSIONS)
            self.set_expression(self.EXPRESSIONS[i])
            self.next_expression = now + 4.0
        blend = 1 - math.exp(-dt * 8)
        for i in range(2):
            self.gaze[i] += (self.gaze_target[i] - self.gaze[i]) * blend
        target = {"sleepy": 0.28, "surprised": 1.2}.get(self.expression, 1.0)
        self.openness += (target - self.openness) * blend
        phase = (now - self.blink_start) / 0.20
        blink = 1 - math.sin(math.pi * phase) if 0 <= phase <= 1 else 1
        self._draw(now - self.started, self.openness * blink)
        self.frame_job = self.root.after(16, self._frame)


MAX_BODY = 24 * 1024 * 1024


def mouth_contours(opening, width=1.0):
    """Return matching w contours; at zero opening they coincide exactly."""
    top, bottom = [], []
    opening = max(0.0, min(1.0, opening))
    for i in range(65):
        u = i / 64
        x = 400 + (u - 0.5) * 88 * width
        # Two smooth U-shaped lobes, joined at a raised center.
        y = 295 + 16 * abs(math.sin(2 * math.pi * u))
        top.append((x, y))
        bottom.append((x, y + 43 * opening * math.sin(math.pi * u)))
    return top, bottom


def wav_envelope(data):
    """Measure audio volume in 20 ms blocks for mouth animation."""
    with wave.open(io.BytesIO(data), "rb") as wav:
        if wav.getsampwidth() != 2 or wav.getcomptype() != "NONE":
            raise ValueError("Expected uncompressed 16-bit PCM WAV")
        channels, rate = wav.getnchannels(), wav.getframerate()
        if channels not in (1, 2) or rate <= 0:
            raise ValueError("Expected mono or stereo WAV")
        samples = array.array("h", wav.readframes(wav.getnframes()))
    if sys.byteorder == "big":
        samples.byteswap()
    block = max(1, round(rate * 0.02)) * channels
    envelope = []
    for i in range(0, len(samples), block):
        chunk = samples[i:i + block]
        envelope.append(math.sqrt(sum(x*x for x in chunk) / len(chunk)) / 32768)
    return envelope, block / channels / rate


class FaceController:
    """All GUI and mixer access stays on the Tk thread."""
    def __init__(self, face):
        self.face = face
        self.commands = queue.Queue(maxsize=32)
        self.alive = True
        self.pygame = None
        self.audio_buffer = None
        self.envelope = []
        self.reference_volume = 0.1
        self.envelope_step = 0.02
        self.silence_threshold = 0.003
        self.job_id = None
        self.speaking = False

    def close(self):
        self.alive = False
        if self.pygame is not None:
            self.pygame.mixer.music.stop()
            self.pygame.mixer.quit()
            self.pygame = None
        self.speaking = False

    def stop(self):
        if self.pygame is not None:
            self.pygame.mixer.music.stop()
        self.speaking = False
        self.face.mouth_open = 0.0
        self.face.mouth_width = 1.0

    def play(self, data):
        audio = base64.b64decode(data["audio"], validate=True)
        envelope, step = wav_envelope(audio)
        if not envelope:
            raise ValueError("TTS produced empty audio")
        if self.pygame is None:
            os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")
            import pygame
            print("[face] Opening audio output...", flush=True)
            pygame.mixer.init(buffer=512)
            print("[face] Audio output ready.", flush=True)
            self.pygame = pygame
        self.stop()
        self.audio_buffer = io.BytesIO(audio)
        self.pygame.mixer.music.load(self.audio_buffer, "wav")
        self.envelope, self.envelope_step = envelope, step
        # A high percentile prevents one brief peak from shrinking every
        # other mouth movement. Quiet speech is still normalized visibly.
        nonzero = sorted(v for v in envelope if v > 0.00001)
        self.reference_volume = max(0.0001, nonzero[int((len(nonzero)-1)*0.9)]
                                    if nonzero else 0.1)
        self.silence_threshold = max(0.00001, self.reference_volume * 0.04)
        self.pygame.mixer.music.play()
        self.speaking = True
        self.job_id = uuid.uuid4().hex
        return {"ok": True, "job": self.job_id, "duration": len(envelope) * step}

    def mouth_pose(self):
        if not self.speaking:
            return (0.0, 1.0)
        if not self.pygame.mixer.music.get_busy():
            self.speaking = False
            return (0.0, 1.0)
        seconds = max(0, self.pygame.mixer.music.get_pos()) / 1000
        block = int(seconds / self.envelope_step)
        if block >= len(self.envelope) or self.envelope[block] < self.silence_threshold:
            return (0.0, 1.0)
        volume = self.envelope[block]
        level = max(0.0, (volume - self.silence_threshold) /
                    max(0.00001, self.reference_volume - self.silence_threshold))
        opening = min(1.0, math.sqrt(level)) * 0.95
        return (opening, 1.0 - 0.12 * opening)

    def dispatch(self, data):
        if not isinstance(data, dict):
            raise ValueError("Expected a JSON command object")
        action = data.get("action")
        if action == "status":
            return {"ok": True, "app": "robot_cat", "protocol": 7, "speaking": self.speaking,
                    "job": self.job_id, "expression": self.face.expression}
        if action == "speak":
            return self.play(data)
        if action == "expression":
            name = data.get("name", "happy")
            if name == "idle":
                name = "happy"
            duration = data.get("duration")
            if duration is not None:
                duration = float(duration)
                if not math.isfinite(duration) or not 0 < duration <= 86400:
                    raise ValueError("Duration must be 0-86400 seconds, excluding zero")
            self.face.demo = False
            self.face.set_expression(name, duration)
        elif action == "blink":
            self.face.blink()
        elif action == "stop":
            self.stop()
        elif action == "fullscreen":
            self.face.fullscreen = not self.face.fullscreen
            self.face.root.attributes("-fullscreen", self.face.fullscreen)
            self.face.canvas.configure(cursor="none" if self.face.fullscreen else "")
        else:
            raise ValueError("Unknown action")
        return {"ok": True}

    def tick(self):
        # Bound command handling per frame so heavy polling cannot starve drawing.
        for _ in range(8):
            try:
                data, response, deadline = self.commands.get_nowait()
            except queue.Empty:
                break
            if time.monotonic() > deadline:
                response.put({"ok": False, "error": "Command expired"})
                continue
            try:
                result = self.dispatch(data)
            except Exception as exc:
                result = {"ok": False, "error": str(exc)}
            response.put(result)


def start_server(controller, port):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def do_POST(self):
            code = 200
            try:
                if self.path != "/command":
                    raise ValueError("Unknown endpoint")
                # This is a local terminal API, not a browser endpoint.
                if self.headers.get("Origin"):
                    raise ValueError("Browser-origin requests are not supported")
                length = int(self.headers.get("Content-Length", "0"))
                if not 0 < length <= MAX_BODY:
                    raise ValueError("Empty or oversized command")
                self.connection.settimeout(10)
                data = json.loads(self.rfile.read(length))
                if not controller.alive:
                    raise ValueError("Face is closing")
                response = queue.Queue(maxsize=1)
                controller.commands.put_nowait((data, response, time.monotonic() + 20))
                result = response.get(timeout=21)
            except Exception as exc:
                code = 400
                result = {"ok": False, "error": str(exc) or "Command timeout / queue full"}
            payload = json.dumps(result).encode()
            try:
                self.send_response(code)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(payload)))
                self.end_headers()
                self.wfile.write(payload)
            except OSError:
                pass  # client closed while this command completed
    server = ThreadingHTTPServer(("127.0.0.1", port), Handler)
    server.daemon_threads = True
    threading.Thread(target=server.serve_forever, daemon=True).start()
    return server


def request(port, action, **fields):
    payload = json.dumps(dict(action=action, **fields)).encode()
    if len(payload) > MAX_BODY:
        raise ValueError("Speech audio is too long; split the text into shorter lines")
    req = Request(f"http://127.0.0.1:{port}/command", data=payload,
                  headers={"Content-Type": "application/json"}, method="POST")
    try:
        with build_opener(ProxyHandler({})).open(req, timeout=25) as response:
            result = json.load(response)
    except HTTPError as exc:
        try:
            message = json.load(exc).get("error", str(exc))
        except (ValueError, AttributeError):
            message = str(exc)
        raise RuntimeError(message) from exc
    except (URLError, OSError) as exc:
        raise RuntimeError(f"Face unavailable on port {port}. Start robot_cat.py first.") from exc
    if not result.get("ok"):
        raise RuntimeError(result.get("error", "Face command failed"))
    return result


class SystemSpeech:
    """Generate PCM WAV with an installed offline speech engine."""
    def __init__(self, voice=None, rate=150):
        self.voice, self.rate = voice, rate
        self.engine = None
        if sys.platform.startswith("linux"):
            self.command = shutil.which("espeak-ng") or shutil.which("espeak")
            if not self.command:
                raise RuntimeError("Install Linux TTS: sudo apt install espeak-ng")
        elif sys.platform == "darwin":
            self.command = "/usr/bin/say"
        elif sys.platform == "win32":
            try:
                import pyttsx3
            except ImportError as exc:
                raise RuntimeError("Install Windows TTS: python -m pip install pyttsx3") from exc
            self.engine = pyttsx3.init(driverName="sapi5")
            self.engine.setProperty("rate", rate)
            if voice is not None:
                self.engine.setProperty("voice", voice)
        else:
            raise RuntimeError("System TTS supports Linux, macOS, and Windows")

    @staticmethod
    def _run(command, text=None):
        try:
            return subprocess.run(command, input=text, text=True, encoding="utf-8",
                                  errors="replace", check=True, capture_output=True,
                                  timeout=30).stdout
        except subprocess.CalledProcessError as exc:
            raise RuntimeError((exc.stderr or "System TTS failed").strip()) from exc
        except subprocess.TimeoutExpired as exc:
            raise RuntimeError("System TTS timed out after 30 seconds. Run: python3 robot_cat.py doctor") from exc

    def voices(self):
        if self.engine is not None:
            for voice in self.engine.getProperty("voices"):
                print(f"{voice.id}  ({voice.name})")
        elif sys.platform == "darwin":
            print(self._run([self.command, "-v", "?"]))
        else:
            print(self._run([self.command, "--voices"]))

    def synthesize(self, text):
        if not text.strip() or len(text) > 1000:
            raise ValueError("Enter 1-1000 characters per utterance")
        with tempfile.TemporaryDirectory(prefix="robot_cat_tts_") as directory:
            path = Path(directory) / "speech.wav"
            if self.engine is not None:
                self.engine.save_to_file(text, str(path))
                self.engine.runAndWait()
            elif sys.platform == "darwin":
                command = [self.command, "-o", str(path), "--file-format=WAVE",
                           "--data-format=LEI16@22050", "-r", str(self.rate), "-f", "-"]
                if self.voice:
                    command.extend(["-v", self.voice])
                self._run(command, text)
            else:
                # A UTF-8 input file avoids any dependence on stdin handling
                # in different eSpeak builds or terminal environments.
                input_path = Path(directory) / "text.txt"
                input_path.write_text(text, encoding="utf-8")
                self._run([self.command, "-v", self.voice or "en", "-s",
                           str(self.rate), "-w", str(path), "-b", "1",
                           "-f", str(input_path)], "")
            if not path.is_file() or path.stat().st_size <= 44:
                raise RuntimeError("TTS produced no audio. Try another installed voice (/voices).")
            audio = path.read_bytes()
            # Validate the format before sending it to the face process.
            with wave.open(io.BytesIO(audio), "rb") as wav:
                if (wav.getsampwidth() != 2 or wav.getcomptype() != "NONE"
                        or wav.getnchannels() not in (1, 2) or wav.getnframes() == 0):
                    raise RuntimeError("The speech voice must produce mono/stereo 16-bit PCM WAV")
            return {"audio": base64.b64encode(audio).decode("ascii")}


class PiperSpeech:
    """Local neural synthesis. Loads existing model files, never downloads."""
    def __init__(self, voice=None, rate=175):
        self.model_dir = Path(__file__).resolve().parent
        if voice:
            model_path = Path(voice).expanduser()
            if model_path.suffix != ".onnx":
                model_path = Path(str(model_path) + ".onnx")
            if not model_path.is_absolute() and not model_path.is_file():
                model_path = self.model_dir / model_path
        else:
            model_path = self.model_dir / "en_US-lessac-medium.onnx"
        self.model_path = model_path.resolve()
        config_path = Path(str(self.model_path) + ".json")
        if not self.model_path.is_file() or not config_path.is_file():
            raise RuntimeError(
                f"Local Piper model/config missing: {self.model_path}. "
                "Place the .onnx and .onnx.json files beside this script, or use "
                "--voice /path/to/voice.onnx. Download links are in the setup "
                "instructions at the top of this script. "
                "Nothing was downloaded automatically.")
        # Some language frontends fetch extra dictionaries/models lazily.
        # Restrict this offline app to self-contained Piper frontends.
        with config_path.open(encoding="utf-8") as config_file:
            model_config = json.load(config_file)
        if model_config.get("phoneme_type", "espeak") not in ("espeak", "text"):
            raise RuntimeError("This offline script supports Piper espeak/text models only; "
                               "other frontends may need extra downloads.")
        try:
            # Disable ONNX Runtime telemetry before importing/loading Piper.
            import onnxruntime
            onnxruntime.disable_telemetry_events()
            from piper import PiperVoice, SynthesisConfig
        except ImportError as exc:
            raise RuntimeError('Install Piper: python3 -m pip install "piper-tts>=1.3,<2"') from exc
        self.command = f"Piper: {self.model_path.name} (offline CPU)"
        print(f"[tts] Loading local model: {self.model_path.name}", flush=True)
        self.voice = PiperVoice.load(str(self.model_path), config_path=str(config_path),
                                     use_cuda=False)
        # Disable optional Arabic diacritizer model downloading as well.
        self.voice.use_tashkeel = False
        self.syn_config = SynthesisConfig(length_scale=175.0 / rate)

    def voices(self):
        paths = {self.model_path}
        for directory in (self.model_dir, self.model_dir / "voices", self.model_path.parent):
            paths.update(directory.glob("*.onnx"))
        for path in sorted(paths):
            if Path(str(path) + ".json").is_file():
                print(path)
        print("These are local files. Select one with controls --voice /path/to/voice.onnx")

    def synthesize(self, text):
        if not text.strip() or len(text) > 1000:
            raise ValueError("Enter 1-1000 characters per utterance")
        buffer = io.BytesIO()
        with wave.open(buffer, "wb") as output:
            self.voice.synthesize_wav(text, output, syn_config=self.syn_config)
        audio = buffer.getvalue()
        with wave.open(io.BytesIO(audio), "rb") as check:
            if check.getnframes() == 0:
                raise RuntimeError("Piper produced empty audio")
        return {"audio": base64.b64encode(audio).decode("ascii")}


def make_speech(voice, rate, tts="piper"):
    if tts == "piper":
        return PiperSpeech(voice, rate)
    if tts == "system":
        return SystemSpeech(voice, rate)
    raise ValueError("Only offline TTS engines (piper, system) are supported")


CONTROL_HELP = """Type any text to speak. Commands:
  /happy [seconds]  /sleepy [seconds]  /curious [seconds]
  /surprised [seconds]  /neutral [seconds]  /idle
  /blink   /stop   /voices   /say TEXT   /help   /quit   /fullscreen
Expression commands stay responsive during speech. /stop clears queued speech.
"""


def speech_worker(commands, port, voice, rate, tts="piper"):
    """TTS owns this process's main thread; terminal input never waits on it."""
    backend = None
    while True:
        kind, text = commands.get()
        try:
            if backend is None:
                print(f"\nStarting {tts} TTS...", flush=True)
                backend = make_speech(voice, rate, tts)
                print(f"[tts] Ready: {getattr(backend, 'command', 'SAPI5')}", flush=True)
            if kind == "voices":
                backend.voices()
                sys.stdout.flush()
                continue
            print("[tts] Generating audio...", flush=True)
            payload = backend.synthesize(text)
            print("[tts] Audio generated; sending to face...", flush=True)
            result = request(port, "speak", **payload)
            print("[tts] Playing. Expression commands remain available.", flush=True)
            deadline = time.monotonic() + float(result["duration"]) + 10
            while True:
                time.sleep(0.1)
                status = request(port, "status")
                if not status["speaking"] or status["job"] != result["job"]:
                    print("[tts] Finished.", flush=True)
                    break
                if time.monotonic() > deadline:
                    request(port, "stop")
                    raise RuntimeError("Audio playback did not finish; run the doctor command")
        except Exception as exc:
            print(f"\nSpeech error: {exc}", flush=True)


class SpeechWorker:
    def __init__(self, args):
        self.args = args
        self.context = multiprocessing.get_context("spawn")
        self.process = None
        self.commands = None

    def submit(self, kind, text=""):
        if self.process is None or not self.process.is_alive():
            self.stop()
            self.commands = self.context.Queue(maxsize=8)
            self.process = self.context.Process(
                target=speech_worker,
                args=(self.commands, self.args.port, self.args.voice, self.args.rate, self.args.tts),
                daemon=True)
            self.process.start()
        try:
            self.commands.put_nowait((kind, text))
        except queue.Full as exc:
            raise ValueError("Speech queue is full. Wait or use /stop.") from exc

    def stop(self):
        if self.process is not None:
            if self.process.is_alive():
                self.process.terminate()
            self.process.join(timeout=2)
            if self.process.is_alive():
                self.process.kill()
                self.process.join()
            self.process.close()
            self.process = None
        if self.commands is not None:
            self.commands.cancel_join_thread()
            self.commands.close()
            self.commands = None


def handle_control(line, args, worker):
    """Return False for /quit; ordinary text is always spoken."""
    line = line.strip()
    if not line:
        return True
    if not line.startswith("/"):
        if len(line) > 1000:
            raise ValueError("Use at most 1000 characters per utterance")
        worker.submit("say", line)
        return True
    command, _, argument = line.partition(" ")
    command = command.lower()
    argument = argument.strip()
    if command == "/say":
        if not argument or len(argument) > 1000:
            raise ValueError("Use /say followed by 1-1000 characters")
        worker.submit("say", argument)
    elif command[1:] in RobotCat.EXPRESSIONS + ("idle",):
        duration = float(argument) if argument else None
        request(args.port, "expression", name=command[1:], duration=duration)
    elif command == "/blink" and not argument:
        request(args.port, "blink")
    elif command == "/stop" and not argument:
        worker.stop()
        request(args.port, "stop")
    elif command == "/voices" and not argument:
        worker.submit("voices")
    elif command == "/help" and not argument:
        print(CONTROL_HELP)
    elif command == "/fullscreen" and not argument:
        request(args.port, "fullscreen")
    elif command in ("/quit", "/exit") and not argument:
        return False
    else:
        raise ValueError("Unknown command or extra arguments. Use /help.")
    return True


def control_terminal(args):
    status = request(args.port, "status")
    if status.get("protocol") != 7:
        raise RuntimeError("An older face process is still running. Close both terminals' "
                           "scripts and restart both using this updated file.")
    worker = SpeechWorker(args)
    print(CONTROL_HELP)
    try:
        while True:
            try:
                if not handle_control(input("control> "), args, worker):
                    break
            except EOFError:
                break
            except KeyboardInterrupt:
                print("\nStopping speech. Use /quit to exit.")
                worker.stop()
                try:
                    request(args.port, "stop")
                except RuntimeError:
                    break
            except (ValueError, RuntimeError) as exc:
                print(f"Error: {exc}")
    finally:
        worker.stop()
        try:
            request(args.port, "stop")
            request(args.port, "expression", name="idle")
        except RuntimeError:
            pass


def display(args):
    root = tk.Tk()
    face = RobotCat(root, args.width, args.height, args.fullscreen,
                    args.hide_help, args.expression)
    controller = FaceController(face)
    face.controller = controller
    try:
        server = start_server(controller, args.port)
    except OSError:
        face.close()
        raise RuntimeError(f"Port {args.port} is already in use; choose another --port")
    try:
        root.mainloop()
    finally:
        controller.close()
        server.shutdown()
        server.server_close()


def doctor_worker(args):
    """A separate process allows the parent to stop a hung native driver."""
    try:
        print(f"[1/4] Starting {args.tts} TTS ({sys.platform})...", flush=True)
        backend = make_speech(args.voice, args.rate, args.tts)
        print(f"      Engine: {getattr(backend, 'command', 'SAPI5')}", flush=True)
        print("[2/4] Generating a test WAV...", flush=True)
        payload = backend.synthesize("Hello. The robot speech test is working.")
        audio = base64.b64decode(payload["audio"])
        with wave.open(io.BytesIO(audio), "rb") as wav:
            duration = wav.getnframes() / wav.getframerate()
        print(f"      Generated {duration:.1f} seconds of audio.", flush=True)
        print("[3/4] Opening audio output...", flush=True)
        os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")
        import pygame
        pygame.mixer.init(buffer=512)
        try:
            sound = pygame.mixer.Sound(file=io.BytesIO(audio))
            print("[4/4] Playing through the default audio output...", flush=True)
            channel = sound.play()
            if channel is None:
                raise RuntimeError("No audio channel is available")
            deadline = time.monotonic() + duration + 5
            while channel.get_busy():
                if time.monotonic() > deadline:
                    raise RuntimeError("Audio playback did not finish")
                time.sleep(0.05)
        finally:
            pygame.mixer.quit()
        print("PASS: synthesis and playback completed. If you heard nothing, check "
              "your Linux output device and volume.", flush=True)
    except Exception as exc:
        print(f"FAIL: {exc}", flush=True)
        raise SystemExit(1)


def diagnose(args):
    context = multiprocessing.get_context("spawn")
    process = context.Process(target=doctor_worker, args=(args,), daemon=True)
    process.start()
    process.join(timeout=45)
    if process.is_alive():
        process.terminate()
        process.join(timeout=2)
        if process.is_alive():
            process.kill()
            process.join()
        process.close()
        raise RuntimeError("Diagnostic timed out. Share the last printed stage to identify "
                           "the stalled TTS/audio component.")
    code = process.exitcode
    process.close()
    if code:
        raise RuntimeError("Speech diagnostic failed; see the stage and error above")


def positive_int(value):
    number = int(value)
    if number <= 0:
        raise argparse.ArgumentTypeError("must be a positive integer")
    return number


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("mode", choices=("display", "controls", "doctor", "run", "speech", "expressions"),
                        default="display", nargs="?")
    parser.add_argument("--width", type=positive_int, default=800)
    parser.add_argument("--height", type=positive_int, default=480)
    parser.add_argument("--fullscreen", action="store_true")
    parser.add_argument("--hide-help", action="store_true", default=True)
    parser.add_argument("--show-help", action="store_false", dest="hide_help")
    parser.add_argument("--expression", choices=RobotCat.EXPRESSIONS, default="happy")
    parser.add_argument("--port", type=positive_int, default=8765)
    parser.add_argument("--tts", choices=("piper", "system"), default="piper",
                        help="piper: offline neural speech; system: offline OS voice")
    parser.add_argument("--voice", help="local .onnx model path for Piper, or system voice ID")
    parser.add_argument("--rate", type=positive_int, default=175)
    args = parser.parse_args()
    if args.port > 65535:
        parser.error("port must be 1-65535")
    try:
        if args.mode in ("display", "run"):
            display(args)
        elif args.mode == "doctor":
            diagnose(args)
        else:
            control_terminal(args)
    except (RuntimeError, tk.TclError, OSError) as exc:
        parser.exit(1, f"Error: {exc}\n")
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
