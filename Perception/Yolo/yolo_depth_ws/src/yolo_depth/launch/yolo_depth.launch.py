from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('yolo_depth')
    params = os.path.join(pkg_share, 'config', 'yolo_depth.yaml')
    return LaunchDescription([
        Node(
            package='yolo_depth',
            executable='yolo_depth_node',
            name='yolo_depth_node',
            parameters=[params],
            output='screen'
        )
    ])
