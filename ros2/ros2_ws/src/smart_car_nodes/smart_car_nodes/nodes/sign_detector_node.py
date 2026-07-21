"""YOLO detection ROS2 adapter with a separate CANN-compatible inference worker."""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage

from smart_car_msgs.msg import SignDetection, SignDetectionArray
from smart_car_nodes.perception_models import PackagedModelPaths
from smart_car_nodes.worker_protocol import InferenceWorkerClient


class SignDetectorNode(Node):
    def __init__(self):
        super().__init__("sign_detector_node")
        self.declare_parameter("model_path", "")
        self.declare_parameter("enabled", True)
        self.declare_parameter("inference_python", "/usr/local/miniconda3/bin/python3")
        self.declare_parameter("score_threshold", 0.05)
        self.score_threshold = float(self.get_parameter("score_threshold").value)
        self.worker = self._load_worker()
        self.publisher = self.create_publisher(SignDetectionArray, "/perception/signs", 10)
        self.create_subscription(CompressedImage, "/camera/image/compressed", self.on_image, qos_profile_sensor_data)
        self.get_logger().info("Sign detector node started.")

    def _load_worker(self):
        if not bool(self.get_parameter("enabled").value):
            return None
        model_path = str(self.get_parameter("model_path").value)
        try:
            resolved = PackagedModelPaths(model_path=model_path, default_filename="yolo.om").resolve_model()
            # Keep CANN/Torch imports out of the ROS2 system-Python process.
            return InferenceWorkerClient(
                "sign",
                str(self.get_parameter("inference_python").value),
                str(resolved),
            )
        except Exception as exc:
            self.get_logger().error(f"Failed to load sign model: {exc}")
            return None

    def on_image(self, msg):
        detections = []
        if self.worker is not None:
            try:
                result = self.worker.infer(bytes(msg.data))
                for x1, y1, x2, y2, label, score in result["detections"]:
                    if float(score) < self.score_threshold:
                        continue
                    detections.append(
                        SignDetection(
                            label=str(label),
                            score=float(score),
                            box=[int(x1), int(y1), int(x2), int(y2)],
                        )
                    )
            except Exception as exc:
                self.get_logger().error(f"Sign inference failed: {exc}")

        out = SignDetectionArray()
        out.header = msg.header
        out.detections = detections
        self.publisher.publish(out)

    def destroy_node(self):
        if self.worker is not None:
            self.worker.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SignDetectorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
