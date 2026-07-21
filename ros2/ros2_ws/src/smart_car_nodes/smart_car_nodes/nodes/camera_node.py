import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage

from smart_car_nodes.camera_probe import open_camera


class CameraNode(Node):
    def __init__(self):
        super().__init__("camera_node")
        self.declare_parameter("camera", 0)
        self.declare_parameter("max_camera_index", 5)
        self.declare_parameter("width", 1280)
        self.declare_parameter("height", 720)
        self.declare_parameter("fps", 30)
        self.declare_parameter("jpeg_quality", 80)

        import cv2

        self.cv2 = cv2
        camera_info = {
            "camera": int(self.get_parameter("camera").value),
            "max_camera_index": int(self.get_parameter("max_camera_index").value),
            "width": int(self.get_parameter("width").value),
            "height": int(self.get_parameter("height").value),
            "fps": int(self.get_parameter("fps").value),
        }
        self.cap, self.camera_index = open_camera(camera_info, cv2_module=cv2, logger=self.get_logger())
        self.jpeg_quality = int(self.get_parameter("jpeg_quality").value)
        self.publisher = self.create_publisher(CompressedImage, "/camera/image/compressed", qos_profile_sensor_data)
        period = 1.0 / max(1, camera_info["fps"])
        self.timer = self.create_timer(period, self.publish_frame)
        self.get_logger().info(f"Camera node started on index {self.camera_index}.")

    def publish_frame(self):
        ret, frame = self.cap.read()
        if not ret or frame is None:
            self.get_logger().warning("Failed to read camera frame.")
            return
        ok, encoded = self.cv2.imencode(
            ".jpg",
            frame,
            [int(self.cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality],
        )
        if not ok:
            self.get_logger().warning("Failed to encode camera frame.")
            return
        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera"
        msg.format = "jpeg"
        msg.data = encoded.tobytes()
        self.publisher.publish(msg)

    def destroy_node(self):
        if hasattr(self, "cap"):
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
