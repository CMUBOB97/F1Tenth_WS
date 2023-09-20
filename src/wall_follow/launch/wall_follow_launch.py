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
            package='wall_follow',
            executable='wall_follow_node',
            name='wall_follow_node',
            parameters=[os.path.join(
                get_package_share_directory('wall_follow'),
                'config', 'wall_follow_params.yaml')],
            output='screen')
        ])

