import threading
import time
from robot_cat_ros.engine import SpeechQueue


def wait_for(predicate):
    deadline = time.monotonic() + 2
    while not predicate():
        assert time.monotonic() < deadline, 'worker timed out'
        time.sleep(0.005)


class ControlledQueue(SpeechQueue):
    def __init__(self, *args, **kwargs):
        self.started = threading.Event()
        self.release = threading.Event()
        self.calls = []
        super().__init__(*args, **kwargs)

    def _synthesize(self, text, language, generation):
        self.calls.append((text, language))
        self.started.set()
        self.release.wait(2)
        return b'fake wav'


def make_worker(capacity=4):
    actions = []
    def request(action, **kwargs):
        actions.append(action)
        return {'job': '1', 'duration': 0.1, 'speaking': False}
    worker = ControlledQueue({'en': {}, 'ja': {}}, request, lambda _: None,
                             capacity=capacity)
    return worker, actions


def test_language_snapshot_and_sequential_playback():
    worker, actions = make_worker()
    try:
        worker.submit('English')
        assert worker.started.wait(1)
        worker.set_language('japanese')
        worker.submit('日本語')
        worker.set_language('en')
        worker.release.set()
        wait_for(lambda: actions.count('speak') == 2)
        assert worker.calls == [('English', 'en'), ('日本語', 'ja')]
        assert actions[:4] == ['speak', 'status', 'speak', 'status']
    finally:
        worker.release.set()
        worker.close()


def test_stop_drops_pending_and_stale_audio():
    worker, actions = make_worker()
    try:
        worker.submit('old')
        assert worker.started.wait(1)
        worker.submit('pending')
        worker.stop()
        worker.release.set()
        worker.submit('new', 'ja')
        wait_for(lambda: actions.count('speak') == 1)
        assert worker.calls == [('old', 'en'), ('new', 'ja')]
    finally:
        worker.release.set()
        worker.close()


def test_queue_capacity_and_invalid_input():
    from unittest import TestCase
    check = TestCase()
    worker, _ = make_worker(capacity=1)
    try:
        worker.submit('active')
        assert worker.started.wait(1)
        worker.submit('pending')
        with check.assertRaisesRegex(ValueError, 'queue full'):
            worker.submit('overflow')
        with check.assertRaises(ValueError):
            worker.submit(' ')
        with check.assertRaises(ValueError):
            worker.set_language('invalid')
    finally:
        worker.stop()
        worker.release.set()
        worker.close()


def test_real_child_io_and_unicode(tmp_path):
    import os
    import sys
    model = tmp_path / 'voice.onnx'
    config = tmp_path / 'config.json'
    model.touch()
    config.write_text('{}')
    executable = tmp_path / 'fake-piper'
    executable.write_text('#!' + sys.executable + '\n'
        'import sys\nfrom pathlib import Path\n'
        'text = sys.stdin.buffer.read()\n'
        'assert text.decode("utf-8") == "こんにちは 世界\\n"\n'
        'assert "--language" in sys.argv\n'
        'assert "--length-scale" in sys.argv\n'
        'Path(sys.argv[sys.argv.index("-f")+1]).write_bytes(b"wav")\n')
    executable.chmod(0o755)
    options = dict(binary=str(executable), model=str(model), config=str(config), length_scale=1.5)
    worker = SpeechQueue({'ja': options}, lambda *a, **kw: {}, lambda _: None)
    try:
        assert worker._synthesize('こんにちは\n世界', 'ja', 0) == b'wav'
    finally:
        worker.close()


def test_priority_warning_replaces_full_queue():
    worker, actions = make_worker(capacity=1)
    try:
        worker.submit('presentation')
        assert worker.started.wait(1)
        worker.submit('queued presentation')
        worker.interrupt('The robot arm is faulty.', 'en')
        worker.release.set()
        wait_for(lambda: actions.count('speak') == 1)
        assert worker.calls == [('presentation', 'en'), ('The robot arm is faulty.', 'en')]
        assert actions[:2] == ['stop', 'speak']
    finally:
        worker.release.set()
        worker.close()


if __name__ == '__main__':
    import tempfile
    from pathlib import Path
    import unittest
    suite = unittest.TestSuite()
    for test in (test_language_snapshot_and_sequential_playback,
                 test_stop_drops_pending_and_stale_audio,
                 test_queue_capacity_and_invalid_input,
                 test_priority_warning_replaces_full_queue):
        suite.addTest(unittest.FunctionTestCase(test))
    def child_test():
        with tempfile.TemporaryDirectory() as directory:
            test_real_child_io_and_unicode(Path(directory))
    suite.addTest(unittest.FunctionTestCase(child_test))
    result = unittest.TextTestRunner(verbosity=2).run(suite)
    raise SystemExit(not result.wasSuccessful())
