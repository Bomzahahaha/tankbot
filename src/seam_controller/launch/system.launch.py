from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        Node(
            package='median_filter',
            executable='weld_detector_median'
        ),

        Node(
            package='seam_controller',
            executable='pid_node'
        ),

        Node(
            package='seam_controller',
            executable='cmd_vel_to_motor'
        ),
    ])
