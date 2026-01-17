import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('tello_uwb'),
        'config',
        'tello_uwb_params.yaml',
    )

    return LaunchDescription([
        Node(
            package='tello_uwb',
            executable='uwb_republisher',
            name='uwb_republisher',
            output='screen',
            parameters=[config],
        ),
    ])
