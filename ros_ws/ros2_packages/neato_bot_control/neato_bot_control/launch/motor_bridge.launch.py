# motor_bridge.launch.py
# ROS 2 launch file to start the Arduino-ROS2 motor bridge node

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='neato_bot_control',
            executable='ros2_motor_bridge',
            name='motor_bridge',
            output='screen',
            parameters=[]  # Add params here if needed later
        )
    ])
