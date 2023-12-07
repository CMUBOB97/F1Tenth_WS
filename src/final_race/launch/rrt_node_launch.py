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
            package='final_race',
            executable='final_race',
            name='final_race',
            parameters=[os.path.join(
                get_package_share_directory('final_race'),
                'config', 'rrt_params.yaml')],
            output='screen')
        ])

