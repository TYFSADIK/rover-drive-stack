"""ros2 launch convoy follower.launch.py   (run alongside the gdn_rover stack: EKF + diff-drive driver)"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package="convoy", executable="convoy_follower_node",
             parameters=[{"serial": "/dev/ttyAMA0", "grace_ms": 4000, "desired_gap_m": 3.0}]),
    ])
