import sys
import threading

import rclpy
from rclpy.node import Node

from smart_car_msgs.msg import CarCommand
from smart_car_nodes.motion import build_manual_command
from smart_car_nodes.ros_messages import command_to_msg


def _read_key():
    try:
        import termios
        import tty
    except ImportError:
        return sys.stdin.readline().strip()

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
        if ch == "\x1b":
            rest = sys.stdin.read(2)
            if rest == "[A":
                return "up"
            if rest == "[B":
                return "down"
            if rest == "[C":
                return "right"
            if rest == "[D":
                return "left"
            return "esc"
        if ch == " ":
            return "space"
        return ch
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


class ManualControlNode(Node):
    def __init__(self):
        super().__init__("manual_control_node")
        self.declare_parameter("speed", 25)
        self.declare_parameter("min_speed", 5)
        self.declare_parameter("max_speed", 60)
        self.speed = int(self.get_parameter("speed").value)
        self.min_speed = int(self.get_parameter("min_speed").value)
        self.max_speed = int(self.get_parameter("max_speed").value)
        self.publisher = self.create_publisher(CarCommand, "/car/command", 10)
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self.keyboard_loop, daemon=True)
        self._thread.start()
        self.get_logger().info("Manual control started. Use w/a/s/d/q/e/j/l/u/p, arrows, space, esc.")

    def keyboard_loop(self):
        while not self._stop.is_set():
            key = _read_key()
            if key == "esc":
                self.publish_key("space")
                rclpy.shutdown()
                break
            if key == "up":
                self.speed = min(self.speed + 1, self.max_speed)
                self.get_logger().info(f"speed={self.speed}")
                continue
            if key == "down":
                self.speed = max(self.speed - 1, self.min_speed)
                self.get_logger().info(f"speed={self.speed}")
                continue
            self.publish_key(key)

    def publish_key(self, key):
        command = build_manual_command(key, speed=self.speed)
        if command is None:
            return
        self.publisher.publish(command_to_msg(command))

    def destroy_node(self):
        self._stop.set()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ManualControlNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
