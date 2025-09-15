from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg = get_package_share_directory('gps')
    defaults = os.path.join(pkg, 'config', 'defaults.yaml')
    return LaunchDescription([
        Node(
            package='gps',
            executable='gps',
            name='gps_viewer',
            parameters=[defaults],
            output='screen'
        )
    ])

