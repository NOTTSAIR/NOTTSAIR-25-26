from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg = get_package_share_directory('camera')
    defaults = os.path.join(pkg, 'config', 'defaults.yaml')

    return LaunchDescription([
        Node(
            package='camera',
            executable='camera',
            name='camera',
            parameters=[defaults],
            output='screen'
        )
    ])

