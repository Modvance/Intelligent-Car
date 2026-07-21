from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("port", default_value=""),
            DeclareLaunchArgument("monitor_port", default_value="8080"),
            DeclareLaunchArgument("camera", default_value="0"),
            DeclareLaunchArgument("speed", default_value="25"),
            Node(
                package="smart_car_nodes",
                executable="esp32_driver_node",
                name="esp32_driver",
                parameters=[{"port": LaunchConfiguration("port"), "motion_enabled_default": True}],
            ),
            Node(
                package="smart_car_nodes",
                executable="camera_node",
                name="camera",
                parameters=[{"camera": ParameterValue(LaunchConfiguration("camera"), value_type=int)}],
            ),
            Node(
                package="smart_car_nodes",
                executable="manual_control_node",
                name="manual_control",
                output="screen",
                parameters=[{"speed": ParameterValue(LaunchConfiguration("speed"), value_type=int)}],
            ),
            Node(
                package="smart_car_nodes",
                executable="monitor_node",
                name="monitor",
                output="screen",
                parameters=[{"port": ParameterValue(LaunchConfiguration("monitor_port"), value_type=int)}],
            ),
        ]
    )
