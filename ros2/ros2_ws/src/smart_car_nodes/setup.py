from glob import glob

from setuptools import find_packages, setup


package_name = "smart_car_nodes"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/weights", glob("weights/*.om")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="student",
    maintainer_email="student@example.com",
    description="ROS2 nodes for the Atlas ESP32 smart car.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "esp32_driver_node = smart_car_nodes.nodes.esp32_driver_node:main",
            "manual_control_node = smart_car_nodes.nodes.manual_control_node:main",
            "camera_node = smart_car_nodes.nodes.camera_node:main",
            "lane_node = smart_car_nodes.nodes.lane_node:main",
            "sign_detector_node = smart_car_nodes.nodes.sign_detector_node:main",
            "decision_node = smart_car_nodes.nodes.decision_node:main",
            "monitor_node = smart_car_nodes.nodes.monitor_node:main",
        ],
    },
)
