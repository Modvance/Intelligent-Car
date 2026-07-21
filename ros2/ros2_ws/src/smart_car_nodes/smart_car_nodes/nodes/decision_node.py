"""ROS2 autonomous-command arbiter for lane, landmark and pedestrian state."""

import time

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool

from smart_car_msgs.msg import CarCommand, DecisionState, LaneResult, SignDetectionArray
from smart_car_nodes.decision_engine import DecisionConfig, DecisionEngine, Detection
from smart_car_nodes.ros_messages import command_to_msg


class DecisionNode(Node):
    def __init__(self):
        super().__init__("decision_node")
        self._declare_parameters()
        self.engine = DecisionEngine(self._read_config())
        self.command_pub = self.create_publisher(CarCommand, "/car/command", 10)
        self.state_pub = self.create_publisher(DecisionState, "/decision/state", 10)
        self.create_subscription(LaneResult, "/perception/lane", self.on_lane, 10)
        self.create_subscription(SignDetectionArray, "/perception/signs", self.on_signs, 10)
        self.create_service(SetBool, "/decision_gate/set_enabled", self.on_set_enabled)
        self.timer = self.create_timer(0.05, self.on_timer)
        enabled = bool(self.get_parameter("enabled").value)
        if enabled:
            self.engine.set_enabled(True, time.monotonic())
        self.get_logger().info(f"Decision node started. enabled={enabled}")

    def _declare_parameters(self):
        defaults = DecisionConfig()
        self.declare_parameter("enabled", False)
        for name in defaults.__dataclass_fields__:
            self.declare_parameter(name, getattr(defaults, name))

    def _read_config(self):
        values = {
            name: self.get_parameter(name).value
            for name in DecisionConfig.__dataclass_fields__
        }
        return DecisionConfig(**values)

    def on_set_enabled(self, request, response):
        self.engine.set_enabled(bool(request.data), time.monotonic())
        response.success = True
        response.message = "decision enabled" if self.engine.enabled else "decision disabled"
        self.get_logger().info(response.message)
        return response

    def on_lane(self, msg):
        self.engine.update_lane(float(msg.filtered_steering))

    def on_signs(self, msg):
        detections = [
            Detection(
                label=str(item.label),
                score=float(item.score),
                box=tuple(int(value) for value in item.box),
            )
            for item in msg.detections
        ]
        self.engine.update_signs(detections, time.monotonic())

    def on_timer(self):
        # The 20 Hz timer, rather than perception callbacks, advances timed actions.
        # This keeps camera inference responsive while a turn or parking sequence runs.
        output = self.engine.tick(time.monotonic())
        command_msg = command_to_msg(output.command)
        command_msg.source = f"decision:{output.status}"
        self.command_pub.publish(command_msg)

        state = DecisionState()
        state.stamp = self.get_clock().now().to_msg()
        state.enabled = bool(self.engine.enabled)
        state.pedestrian_blocked = bool(output.pedestrian_blocked)
        state.action_active = bool(self.engine.active_action)
        state.status = output.status
        state.action = output.action
        state.turn_count = int(output.turn_count)
        state.right_turn_count = int(output.right_turn_count)
        state.turnaround_count = int(self.engine.turnaround_count)
        self.state_pub.publish(state)


def main(args=None):
    rclpy.init(args=args)
    node = DecisionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
