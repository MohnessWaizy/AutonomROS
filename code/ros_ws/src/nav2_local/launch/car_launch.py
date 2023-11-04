import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import PushRosNamespace
from launch_ros.actions import Node


def generate_launch_description():
    # Get the launch directory
    local_dir = get_package_share_directory('nav2_local')
    launch_dir = os.path.join(local_dir, 'launch')
    

    # Create the launch configuration variables
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    map_yaml_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')

    slash_urdf = os.path.join(
        get_package_share_directory('slash_description'),
        'urdf',
        'slash.urdf'
    )

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    declare_use_namespace_cmd = DeclareLaunchArgument(
        'use_namespace',
        default_value='false',
        description='Whether to apply a namespace to the navigation stack')

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(local_dir, 'maps', 'simplestreet3.yaml'),
        description='Full path to map yaml file to load')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(local_dir, 'params', 'nav2_car_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='false',
        description='Automatically startup the nav2 stack')
    
    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config', default_value=os.path.join(local_dir, 'rviz', 'rviz_launch_config.rviz'),
        description='Full path to rviz config file')

    # Specify the actions
    bringup_cmd_group = GroupAction([
        PushRosNamespace(
            condition=IfCondition(use_namespace),
            namespace=namespace),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir,
                                                       'localization_launch.py')),
            launch_arguments={'namespace': namespace,
                              'map': map_yaml_file,
                              'use_sim_time': use_sim_time,
                              'autostart': autostart,
                              'params_file': params_file,
                              'use_lifecycle_mgr': 'false'}.items()),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'navigation_launch.py')),
            launch_arguments={'namespace': namespace,
                              'use_sim_time': use_sim_time,
                              'autostart': autostart,
                              'params_file': params_file,
                              'use_lifecycle_mgr': 'false',
                              'map_subscribe_transient_local': 'true'}.items()),
        
    
    ])
    
    load_nodes = GroupAction(
        actions=[
            Node(
                package = "tf2_ros", 
                executable = "static_transform_publisher",
                # arguments = ["0", "-0.35", "0", "0", "0", "0", "map", "odom"],
                arguments = ["0.96", "-2.18", "0", "0", "0", "0", "map", "odom"]
            ),
            Node(
                package='astra_pro',
                executable='astra_pro_publisher',
                name='astra_pro_publisher',
                output='screen',
                parameters=[params_file]),
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
                package = 'bno055',
                executable = 'bno055',
                parameters = [params_file]
            ),
            Node(
                package='robot_localization',
                executable='ekf_node',
                name='ekf_filter_node',
                output='screen',
                parameters=[params_file],
            ),
            Node(
                package='carcontroller',
                executable='carcontroller',
                parameters=[params_file],
                output='screen'
            ),
            Node(
                package='camera',
                executable='cam2image',
                parameters=[params_file],
                output='screen'
            )
        ]
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    
    ld.add_action(load_nodes)

    # Add the actions to launch all of the navigation nodes
    ld.add_action(bringup_cmd_group)

    return ld
