# Copyright 2021 the Autoware Foundation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import os


def generate_launch_description():
    """
    Launch lane planning nodes.

    """
    local_dir = get_package_share_directory('autonomros_lane_detection')

    DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(local_dir, 'params', 'ost.yaml'),
            description='Full path to the ROS2 parameters file to use'),
    

    params_file = LaunchConfiguration('params_file')

    
    # Nodes

    lane_detector_node = Node(
        package='autonomros_lane_detection',
        executable='lane_detector',
        arguments=['--ros-args', '--log-level', 'info'],
        remappings=[
            # ("camera/image", "camera_sensor1/image_raw")
            ("/camera_sensor1/image_raw", "/image")
        ],
        parameters=[
            # {"intrinsics_path": "install/autonomros_lane_detection/share/autonomros_lane_detection/params/intrinsics.yml"},
            # {"pointsets_path": "install/autonomros_lane_detection/share/autonomros_lane_detection/params/pointsets.yml"}
            {"intrinsics_path": "/home/xilinx/ros/install_aarch64/share/autonomros_lane_detection/params/real_intrinsics.yml"},
            {"pointsets_path": "/home/xilinx/ros/install_aarch64/share/autonomros_lane_detection/params/real_pointsets.yml"}
        ]
    )



    return LaunchDescription([
        lane_detector_node
    ])