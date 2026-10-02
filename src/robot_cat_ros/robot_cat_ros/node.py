import queue
from functools import partial
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String, Empty

from .engine import SpeechQueue
from .face import request, RobotCat
from .presenter import PresenterBridge


class SpeechNode(Node):
    def __init__(self):
        super().__init__('robot_cat_speech')
        defaults = {
            'face_port': 8765, 'language': 'en', 'queue_size': 4,
            'synthesis_timeout': 60.0,
            'english_binary': 'piper',
            'english_model': '~/robot_voices/en_US-lessac-medium.onnx',
            'english_config': '~/robot_voices/en_US-lessac-medium.onnx.json',
            'english_length_scale': 1.0,
            'japanese_binary': 'piper',
            'japanese_model': '~/robot_voices/ja_JP-hi_fi_captain-medium.onnx',
            'japanese_config': '~/robot_voices/ja_JP-hi_fi_captain-medium.onnx.json',
            'japanese_length_scale': 1.0,
            'presenter_language': 'auto',
            'presenter_text_override': '',
            'fault_text': 'The robot arm is faulty.',
            'fault_language': 'en',
        }
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        values = {key: self.get_parameter(key).value for key in defaults}
        if values['queue_size'] < 1 or values['synthesis_timeout'] <= 0:
            raise ValueError('queue_size and synthesis_timeout must be positive')
        self.events = queue.Queue()
        self.actions = queue.Queue(maxsize=16)
        self.finished = threading.Event()
        self.face_request = partial(request, values['face_port'])
        configs = {}
        for code, name in [('en', 'english'), ('ja', 'japanese')]:
            configs[code] = {field: values[name + '_' + field]
                             for field in ('binary', 'model', 'config', 'length_scale')}
        self.worker = SpeechQueue(configs, self.face_request, self.events.put,
                                  values['queue_size'], values['synthesis_timeout'])
        self.worker.set_language(values['language'])
        self.presenter = PresenterBridge(
            self.worker, self.events.put, values['presenter_language'],
            values['presenter_text_override'], values['fault_text'], values['fault_language'])
        # Volatile: old latched speech is never replayed when starting this node.
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE)
        self.status = self.create_publisher(String, '/robot_cat/status', qos)
        # Compatible with reliable OR best-effort health publishers. Volatile
        # also accepts live updates from a transient-local publisher.
        health_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                               durability=DurabilityPolicy.VOLATILE)
        self.subscriptions_ = [
            self.create_subscription(String, '/tts/say', self.presenter_say, qos),
            self.create_subscription(Empty, '/presenter_abort', self.presenter_abort, qos),
            self.create_subscription(String, '/arm_health_state', self.arm_health, health_qos),
            self.create_subscription(String, '/presenter_state', self.presenter_state, health_qos),
            self.create_subscription(String, '/robot_cat/say', self.say, qos),
            self.create_subscription(String, '/robot_cat/say/en',
                                     lambda msg: self.say(msg, 'en'), qos),
            self.create_subscription(String, '/robot_cat/say/ja',
                                     lambda msg: self.say(msg, 'ja'), qos),
            self.create_subscription(String, '/robot_cat/language', self.language, qos),
            self.create_subscription(String, '/robot_cat/command', self.command, qos),
        ]
        self.create_timer(0.05, self.flush_events)
        self.action_thread = threading.Thread(target=self.action_loop, daemon=True)
        self.action_thread.start()
        self.get_logger().info('Ready: /tts/say, /presenter_abort, /arm_health_state, '
                               '/presenter_state; manual language=' + self.worker.language)

    def presenter_say(self, message):
        try:
            self.presenter.say(message.data)
        except ValueError as exc:
            self.events.put('error: ' + str(exc))

    def presenter_abort(self, _message):
        self.presenter.abort()

    def arm_health(self, message):
        self.presenter.health(message.data)

    def presenter_state(self, message):
        self.presenter.presenter_state(message.data)

    def say(self, message, language=None):
        try:
            self.worker.submit(message.data, language)
        except ValueError as exc:
            self.events.put('error: ' + str(exc))

    def language(self, message):
        try:
            self.worker.set_language(message.data)
            self.events.put('language:' + self.worker.language)
        except ValueError as exc:
            self.events.put('error: ' + str(exc))

    def command(self, message):
        command = message.data.strip().lower().lstrip('/')
        if command in ('english', 'japanese', 'en', 'ja'):
            self.language(String(data=command))
            return
        if command not in RobotCat.EXPRESSIONS + ('idle', 'blink', 'fullscreen', 'stop'):
            self.events.put('error: unknown face command ' + command)
            return
        if command == 'stop':
            self.worker.interrupt()
            return
        try:
            self.actions.put_nowait(command)
        except queue.Full:
            self.events.put('error: face command queue full')

    def action_loop(self):
        while not self.finished.is_set():
            try:
                command = self.actions.get(timeout=0.1)
            except queue.Empty:
                continue
            try:
                if command == 'stop':
                    self.worker.stop()
                elif command in ('blink', 'fullscreen'):
                    self.face_request(command)
                else:
                    self.face_request('expression', name=command)
            except Exception as exc:
                self.events.put('error: ' + str(exc))

    def flush_events(self):
        for _ in range(32):
            try:
                event = self.events.get_nowait()
            except queue.Empty:
                return
            self.status.publish(String(data=event))
            if event.startswith('error:'):
                self.get_logger().error(event)

    def close(self):
        self.finished.set()
        self.action_thread.join(timeout=26)
        self.worker.close()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = SpeechNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
