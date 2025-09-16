from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg = get_package_share_directory('imu')
    defaults = os.path.join(pkg, 'config', 'defaults.yaml')

    topic = LaunchConfiguration('topic')
    refresh_hz = LaunchConfiguration('refresh_hz')

    return LaunchDescription([
        DeclareLaunchArgument('topic', default_value='/imu/data'),
        DeclareLaunchArgument('refresh_hz', default_value='15.0'),

        Node(
            package='imu',
            executable='imu',
            name='imu_viewer',
            parameters=[defaults, {
                'topic': topic,
                'refresh_hz': refresh_hz
            }],
            output='screen'
        )
    ])
