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
            package='gap_follow',
            executable='reactive_node',
            name='reactive_node',
            parameters=[os.path.join(
                get_package_share_directory('gap_follow'),
                'config', 'gap_follow_params.yaml')],
            output='screen')
        ])

