#!$HOME/env/bin/python

import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='piplane',
            executable='crsf_node',
            name='crsf_node'
        ),
        Node(
            package='piplane',
            executable='Controller',
            name='Controller'
        ),
        Node(
            package='piplane',
            executable='imu_publisher',
            name='imu_publisher')
        ),
        # Something is wrong with MicroRos when launched here, will investigate
        Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            name='micro_ros_agent',
            output='screen',
            parameters=[{'discovery_address': 'serial', 'dev': '/dev/ttyACM0'}]  # Adjust parameters as needed
        )
    ])

if __name__ == '__main__':
    generate_launch_description()
