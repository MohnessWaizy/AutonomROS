import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(
                get_package_share_directory('astra_pro'),
                'params',
                'params.yaml'
            )
        ),
        Node(
            package='astra_pro',
            executable='astra_pro_publisher',
            parameters=[LaunchConfiguration('params_file')],
            output='screen'
        )
    ])
