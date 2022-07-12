from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros_qwiic_dual_encoder',
            executable='ros_qwiic_dual_encoder',
            name='ros_qwiic_dual_encoder',
            output='screen'),
    ])