from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('yolo_infer')
    defaults = os.path.join(pkg_share, 'config', 'defaults.yaml')

    return LaunchDescription([
        Node(
            package='yolo_infer',
            executable='yolo_node',
            name='yolo_node',
            parameters=[defaults],
            output='screen'
        )
    ])

