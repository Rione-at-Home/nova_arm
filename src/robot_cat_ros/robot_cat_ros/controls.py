"""Interactive controls plus the speech subscriber in the same terminal."""
import queue
import threading
import rclpy
from std_msgs.msg import String
from .node import SpeechNode

HELP = '''Text speaks in the selected language.
/english /japanese   select language (queued messages retain their language)
/happy /neutral /sleepy /curious /surprised /idle
/fullscreen /blink /stop
/help /quit
ROS messages are accepted while this terminal is open.'''


def main(args=None):
    rclpy.init(args=args)
    node = SpeechNode()
    lines = queue.Queue(maxsize=32)

    def read_input():
        try:
            while True:
                lines.put(input('control> '))
        except EOFError:
            lines.put('/quit')

    def tick():
        try:
            line = lines.get_nowait().strip()
        except queue.Empty:
            return
        command, _, argument = line.partition(' ')
        command = command.lower()
        if command in ('/quit', '/exit'):
            raise KeyboardInterrupt
        if command == '/help':
            print(HELP)
        elif command == '/say':
            node.say(String(data=argument))
        elif line.startswith('/'):
            node.command(String(data=line))
        elif line:
            node.say(String(data=line))

    node.create_timer(0.05, tick)
    threading.Thread(target=read_input, daemon=True).start()
    print(HELP)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
