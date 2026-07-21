"""LaneNet ROS2 adapter; model inference stays in the validated Python 3.9 worker."""

import rclpy
import math
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage

from smart_car_msgs.msg import LaneLine, LaneResult
from smart_car_nodes.lane_geometry import (
    combine_steering,
    compute_lateral_steering,
    compute_steering_command,
    compute_turn_control,
    transform_lane_coords,
)
from smart_car_nodes.perception_models import PackagedModelPaths
from smart_car_nodes.worker_protocol import InferenceWorkerClient


class LaneNode(Node):
    def __init__(self):
        super().__init__("lane_node")
        self.declare_parameter("model_path", "")
        self.declare_parameter("enabled", True)
        self.declare_parameter("inference_python", "/usr/local/miniconda3/bin/python3")
        self.declare_parameter("alpha", 0.8)
        self.alpha = float(self.get_parameter("alpha").value)
        self.prev_steering = None
        self.worker = self._load_worker()
        self.publisher = self.create_publisher(LaneResult, "/perception/lane", 10)
        self.create_subscription(CompressedImage, "/camera/image/compressed", self.on_image, qos_profile_sensor_data)
        self.get_logger().info("Lane node started.")

    def _load_worker(self):
        if not bool(self.get_parameter("enabled").value):
            return None
        model_path = str(self.get_parameter("model_path").value)
        try:
            resolved = PackagedModelPaths(model_path=model_path, default_filename="lanenet.om").resolve_model()
            # ROS2 Humble uses system Python 3.10, while CANN inference uses Python 3.9.
            return InferenceWorkerClient(
                "lane",
                str(self.get_parameter("inference_python").value),
                str(resolved),
            )
        except Exception as exc:
            self.get_logger().error(f"Failed to load lane model: {exc}")
            return None

    def on_image(self, msg):
        lanes = []
        inference_time_ms = 0.0
        if self.worker is not None:
            try:
                result = self.worker.infer(bytes(msg.data))
                w = int(result["image_width"])
                h = int(result["image_height"])
                raw_lanes = sorted(result["lanes"], key=lambda item: item[2], reverse=True)[:2]
                lanes = transform_lane_coords(
                    raw_lanes,
                    (result["model_width"], result["model_height"]),
                    (w, h),
                )
                inference_time_ms = float(result["inference_time_ms"])
            except Exception as exc:
                self.get_logger().error(f"Lane inference failed: {exc}")
                return
        else:
            return

        heading = compute_steering_command(lanes, w, h)
        lateral, lateral_error, lane_center, lane_target = compute_lateral_steering(lanes, w, h)
        steering = combine_steering(heading, lateral)
        if self.prev_steering is None:
            filtered = steering
        else:
            filtered = self.alpha * steering + (1.0 - self.alpha) * self.prev_steering
        self.prev_steering = steering
        should_turn, turn_strength = compute_turn_control(filtered)
        if not should_turn:
            intended_action = "advance"
        elif filtered > 0:
            intended_action = "turn_right"
        else:
            intended_action = "turn_left"

        out = LaneResult()
        out.header = msg.header
        out.lanes = [LaneLine(k=float(k), b=float(b), confidence=float(conf)) for k, b, conf in lanes]
        out.steering_command = float(steering)
        out.filtered_steering = float(filtered)
        out.inference_time_ms = float(inference_time_ms)
        out.heading_steering = float(heading)
        out.lateral_steering = float(lateral)
        out.lateral_error_px = float("nan") if lateral_error is None else float(lateral_error)
        out.lane_center_x = float("nan") if lane_center is None else float(lane_center)
        out.lane_target_x = float(lane_target)
        out.turn_strength = float(turn_strength)
        out.intended_action = intended_action
        self.publisher.publish(out)

    def destroy_node(self):
        if self.worker is not None:
            self.worker.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LaneNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
