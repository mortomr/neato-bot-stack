#motor_bridge.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='neato_bot_control',
            executable='cmd_vel_to_pwm',
            name='cmd_vel_to_pwm',
            output='screen'
        )
    ])

