# TODO: check styling

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    slash_urdf = os.path.join(
        get_package_share_directory('slash_description'),
        'urdf',
        'slash.urdf'
    )
    rviz_config = os.path.join(
        get_package_share_directory('slash_description'),
        'rviz',
        'slash.rviz'
    )

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            arguments=[slash_urdf]
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
        )
    ])
