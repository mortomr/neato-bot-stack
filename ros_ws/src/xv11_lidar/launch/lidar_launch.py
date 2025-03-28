from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='xv11_lidar',
            executable='xv11_lidar',
            name='xv11_lidar',
            output='screen',
            parameters=[
                {'port': '/dev/ttyUSB0'},
                {'frame_id': 'xv11_lidar'},
                {'range_min': 0.06},
                {'range_max': 6.0}
            ]
        )
    ])
