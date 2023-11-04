# TODO: check styling

import os
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    slash_urdf = PathJoinSubstitution([
        FindPackageShare('slash_description'),
        'urdf',
        'slash.urdf'
    ])
    world_file = PathJoinSubstitution([
        FindPackageShare('slash_description'),
        'worlds',
        'teststreet.world'
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value=world_file,
            description='Specify world file name'
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('gazebo_ros'),
                    'launch',
                    'gazebo.launch.py'
                ])
            ])
        ),
        Node(
            package='robot_state_publisher',
            node_executable='robot_state_publisher',
            arguments=[slash_urdf]
        ),
        Node(
            package='gazebo_ros',
            node_executable='spawn_entity.py',
            arguments=[
                '-entity',
                'slash',
                '-x',
                '-3.5',
                '-y'
                '-0.83',
                '-file',
                slash_urdf
            ],
            output='screen'
        )
    ])
