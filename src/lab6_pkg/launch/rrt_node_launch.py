# launch file for:
#  - getting parameters in yaml file
#  - launching safety node

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='lab6_pkg',
            executable='rrt_node',
            name='rrt_node',
            parameters=[os.path.join(
                get_package_share_directory('lab6_pkg'),
                'config', 'rrt_params.yaml')],
            output='screen')
        ])

