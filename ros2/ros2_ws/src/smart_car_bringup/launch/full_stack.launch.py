import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config = os.path.join(get_package_share_directory("smart_car_bringup"), "config", "full_stack.yaml")
    return LaunchDescription(
        [
            DeclareLaunchArgument("port", default_value=""),
            DeclareLaunchArgument("lane_model_path", default_value=""),
            DeclareLaunchArgument("sign_model_path", default_value=""),
            DeclareLaunchArgument("monitor_port", default_value="8080"),
            DeclareLaunchArgument("camera", default_value="0"),
            DeclareLaunchArgument("inference_python", default_value="/usr/local/miniconda3/bin/python3"),
            DeclareLaunchArgument("decision_enabled", default_value="false"),
            Node(
                package="smart_car_nodes",
                executable="esp32_driver_node",
                name="esp32_driver",
                output="screen",
                parameters=[config, {"port": LaunchConfiguration("port"), "motion_enabled_default": False}],
            ),
            Node(
                package="smart_car_nodes",
                executable="camera_node",
                name="camera",
                output="screen",
                parameters=[config, {"camera": ParameterValue(LaunchConfiguration("camera"), value_type=int)}],
            ),
            Node(
                package="smart_car_nodes",
                executable="lane_node",
                name="lane",
                output="screen",
                parameters=[
                    config,
                    {
                        "model_path": LaunchConfiguration("lane_model_path"),
                        "inference_python": LaunchConfiguration("inference_python"),
                    },
                ],
            ),
            Node(
                package="smart_car_nodes",
                executable="sign_detector_node",
                name="sign_detector",
                output="screen",
                parameters=[
                    config,
                    {
                        "model_path": LaunchConfiguration("sign_model_path"),
                        "inference_python": LaunchConfiguration("inference_python"),
                    },
                ],
            ),
            Node(
                package="smart_car_nodes",
                executable="decision_node",
                name="decision",
                output="screen",
                parameters=[config, {"enabled": ParameterValue(LaunchConfiguration("decision_enabled"), value_type=bool)}],
            ),
            Node(
                package="smart_car_nodes",
                executable="monitor_node",
                name="monitor",
                output="screen",
                parameters=[config, {"port": ParameterValue(LaunchConfiguration("monitor_port"), value_type=int)}],
            ),
        ]
    )
