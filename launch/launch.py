#!/home/user/env/bin/python

import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.substitutions import FindExecutable, LaunchConfiguration


def generate_launch_description():
    micro_ros_port = "/dev/ttyACM0"
    respawn_time = 0.1
    micro_ros_baudrate = 115200
    return LaunchDescription(
        [
            Node(package="piplane", executable="crsf_node", name="crsf_node"),
            Node(package="piplane", executable="imu_publisher", name="imu_publisher"),
            Node(package="piplane", executable="Controller", name="Controller"),
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "run",
                    "micro_ros_agent",
                    "micro_ros_agent",
                    "serial",
                    "--dev",
                    micro_ros_port,
                    f"baudrate:={micro_ros_baudrate}",
                ],
                shell=True,
                name="micro-ros-agent",
                output="both",
                respawn=True,
                respawn_delay=respawn_time,
            ),
        ]
    )


if __name__ == "__main__":
    generate_launch_description()
