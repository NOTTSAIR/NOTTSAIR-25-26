from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('static_a')
    params = os.path.join(pkg_share, 'config', 'static_a.yaml')

    return LaunchDescription([
        Node(
            package='static_a',
            executable='static_inspection_a',   # must match add_executable()
            name='static_a',
            output='screen',
            parameters=[params]                 # remove if you don’t use params
        )
    ])

