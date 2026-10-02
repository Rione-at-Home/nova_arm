"""Bounded speech queue; no ROS dependency and no shell execution."""
import base64
from collections import deque
import os
from pathlib import Path
import subprocess
import tempfile
import threading
import time


class SpeechQueue:
    def __init__(self, configs, face_request, report, capacity=4, timeout=60.0):
        self.configs, self.request, self.report = configs, face_request, report
        self.capacity, self.timeout = capacity, timeout
        self.language = 'en'
        self.pending = deque()
        self.cv = threading.Condition(threading.RLock())
        self.playback_lock = threading.Lock()
        self.generation = 0
        self.closed = False
        self.process = None
        self.stop_pending = False
        self.thread = threading.Thread(target=self._work, daemon=True)
        self.thread.start()

    def set_language(self, language):
        aliases = {'english': 'en', 'japanese': 'ja', 'en': 'en', 'ja': 'ja'}
        language = aliases.get(language.strip().lower())
        if not language:
            raise ValueError('Language must be en or ja')
        with self.cv:
            self.language = language

    def submit(self, text, language=None):
        text = text.strip()
        if not 1 <= len(text) <= 1000:
            raise ValueError('Speech must contain 1-1000 characters')
        with self.cv:
            if self.closed:
                raise ValueError('Speech worker is closed')
            language = language or self.language
            if language not in self.configs:
                raise ValueError('Language must be en or ja')
            if len(self.pending) >= self.capacity:
                raise ValueError('Speech queue full; newest message rejected')
            self.pending.append((text, language, self.generation))
            self.cv.notify_all()

    def interrupt(self, text=None, language=None):
        """Cancel immediately, with playback stop serviced by the worker thread.

        Optional replacement speech bypasses a full normal queue. Safe to call
        from a ROS callback: no HTTP requests or process waits here.
        """
        if text is not None and (not text.strip() or len(text) > 1000):
            raise ValueError('Replacement speech must contain 1-1000 characters')
        with self.cv:
            if text is not None and (language or self.language) not in self.configs:
                raise ValueError('Language must be en or ja')
            self.generation += 1
            self.pending.clear()
            if self.process and self.process.poll() is None:
                try:
                    self.process.kill()
                except ProcessLookupError:
                    pass
            self.stop_pending = True
            if text is not None and not self.closed:
                self.pending.append((text.strip(), language or self.language, self.generation))
            self.cv.notify_all()

    def stop(self):
        self.interrupt()

    def _stop_playback(self):
        # Network waits never hold the queue lock used by ROS callbacks.
        with self.playback_lock:
            try:
                self.request('stop')
            except Exception as exc:
                self.report('error: ' + str(exc))

    def close(self):
        with self.cv:
            self.closed = True
        self.stop()
        self.thread.join(timeout=30)

    def _current(self, generation):
        return not self.closed and generation == self.generation

    def _synthesize(self, text, language, generation):
        config = self.configs[language]
        model = Path(config['model']).expanduser()
        settings = Path(config['config']).expanduser()
        if not model.is_file() or not settings.is_file():
            raise ValueError(f'{language}: local model/config missing: {model}, {settings}')
        with tempfile.TemporaryDirectory(prefix='robot-cat-') as directory:
            output = Path(directory) / 'speech.wav'
            command = [os.path.expanduser(config['binary']), '--model', str(model),
                       '--config', str(settings), '-f', str(output),
                       '--length-scale', str(config['length_scale'])]
            # Both CLIs accept UTF-8 text on stdin. No shell or automatic downloads.
            env = dict(os.environ, CUDA_VISIBLE_DEVICES='', OMP_NUM_THREADS='2',
                       OPENBLAS_NUM_THREADS='2')
            with self.cv:
                if not self._current(generation):
                    return None
                self.process = subprocess.Popen(command, stdin=subprocess.PIPE,
                                                stdout=subprocess.PIPE,
                                                stderr=subprocess.PIPE, env=env)
                process = self.process
            try:
                _, errors = process.communicate((' '.join(text.splitlines()) + '\n').encode('utf-8'),
                                                timeout=self.timeout)
            except subprocess.TimeoutExpired:
                process.kill()
                process.communicate()
                raise RuntimeError('Speech synthesis timed out')
            finally:
                with self.cv:
                    self.process = None
            if not self._current(generation):
                return None
            if process.returncode:
                raise RuntimeError(errors.decode('utf-8', errors='replace')[-1500:])
            return output.read_bytes()

    def _work(self):
        while True:
            with self.cv:
                self.cv.wait_for(lambda: self.closed or self.stop_pending or self.pending)
                stop = self.stop_pending
                self.stop_pending = False
                closed = self.closed
                if not stop and not closed:
                    text, language, generation = self.pending.popleft()
            if stop:
                self._stop_playback()
                if not closed:
                    continue
            if closed:
                return
            try:
                self.report('synthesizing:' + language)
                audio = self._synthesize(text, language, generation)
                with self.playback_lock:
                    if audio is None or not self._current(generation):
                        continue
                    result = self.request('speak', audio=base64.b64encode(audio).decode('ascii'))
                self.report('speaking:' + language)
                deadline = time.monotonic() + float(result['duration']) + 10
                while self._current(generation):
                    if time.monotonic() > deadline:
                        self.request('stop')
                        raise RuntimeError('Playback timed out')
                    status = self.request('status')
                    if not status.get('speaking') or status.get('job') != result.get('job'):
                        break
                    with self.cv:
                        self.cv.wait(timeout=0.05)
                self.report('idle')
            except Exception as exc:
                self.report('error: ' + str(exc))
