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
            package='planning_project',
            executable='planning_project',
            name='planning_project',
            parameters=[os.path.join(
                get_package_share_directory('planning_project'),
                'config', 'rrt_params.yaml')],
            output='screen')
        ])

