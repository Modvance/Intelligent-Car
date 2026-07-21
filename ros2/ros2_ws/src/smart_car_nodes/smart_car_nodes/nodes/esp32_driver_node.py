"""ROS2 node that is the sole owner of the ESP32 serial port."""

import time

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool

from smart_car_msgs.msg import CarCommand, CarState, MotorTelemetry
from smart_car_nodes.motion import MotionGate
from smart_car_nodes.protocol import CHECK_VALUE, CommandFrame, DEFAULT_SERVO
from smart_car_nodes.ros_messages import fill_car_state_msg, msg_to_command
from smart_car_nodes.serial_driver import Esp32SerialClient, SerialConfig


class Esp32DriverNode(Node):
    def __init__(self):
        super().__init__("esp32_driver_node")
        self.declare_parameter("port", "")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("esp32_name", "1a86_USB_Serial")
        self.declare_parameter("motion_enabled_default", True)

        self.gate = MotionGate(bool(self.get_parameter("motion_enabled_default").value))
        self.client = Esp32SerialClient(
            SerialConfig(
                port=str(self.get_parameter("port").value),
                baudrate=int(self.get_parameter("baudrate").value),
                esp32_name=str(self.get_parameter("esp32_name").value),
            )
        )

        self.state_pub = self.create_publisher(CarState, "/car/state", 10)
        self.telemetry_pub = self.create_publisher(MotorTelemetry, "/car/motor_telemetry", 10)
        self.create_subscription(CarCommand, "/car/command", self.on_command, 10)
        self.create_service(SetBool, "/motion_gate/set_enabled", self.on_set_motion_enabled)
        self.create_timer(0.05, self.publish_telemetry)
        self.get_logger().info("ESP32 driver node started.")

    def on_set_motion_enabled(self, request, response):
        self.gate.set_enabled(request.data)
        response.success = True
        response.message = "motion enabled" if self.gate.enabled else "motion disabled"
        self.get_logger().info(response.message)
        return response

    def on_command(self, msg):
        requested = msg_to_command(msg)
        # MotionGate only zeros wheel PWM; servo commands remain valid for a safe pose.
        command = self.gate.apply(requested)
        start = time.time()
        try:
            result = self.client.send(command) or "NO_RESPONSE"
        except Exception as exc:
            result = f"ERROR: {exc}"
            self.get_logger().error(result)
        latency_ms = (time.time() - start) * 1000.0
        state_msg = fill_car_state_msg(
            CarState(),
            self,
            command,
            result,
            self.client.port,
            self.gate.enabled,
            latency_ms,
        )
        self.state_pub.publish(state_msg)

    def publish_telemetry(self):
        try:
            frames = self.client.drain_telemetry()
        except Exception:
            return
        for frame in frames:
            msg = MotorTelemetry()
            msg.stamp = self.get_clock().now().to_msg()
            msg.sequence = int(frame["sequence"])
            msg.period_ms = int(frame["period_ms"])
            msg.fresh = True
            for index, wheel in enumerate(frame["wheels"]):
                msg.target_rpm[index] = float(wheel["target_rpm"])
                msg.measured_rpm[index] = float(wheel["measured_rpm"])
                msg.tick_delta[index] = int(wheel["tick_delta"])
                msg.pwm[index] = int(wheel["pwm"])
                msg.error[index] = float(wheel["error"])
                msg.p_term[index] = float(wheel["p_term"])
                msg.i_term[index] = float(wheel["i_term"])
                msg.d_term[index] = float(wheel["d_term"])
                msg.flags[index] = int(wheel["flags"])
            self.telemetry_pub.publish(msg)

    def destroy_node(self):
        try:
            # Always leave motors stopped and return the camera to its calibrated pose.
            stop = CommandFrame(motor=[0, 0, 0, 0], servo=DEFAULT_SERVO, check_value=CHECK_VALUE, source="shutdown")
            self.client.send(stop)
        except Exception:
            pass
        self.client.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Esp32DriverNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
