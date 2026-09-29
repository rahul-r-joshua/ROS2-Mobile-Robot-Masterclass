#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('mobile_robot_controller')
    default_params = os.path.join(pkg_share, 'config', 'controller_params.yaml')

    # Launch Arguments
    use_cpp_arg = DeclareLaunchArgument(
        'use_cpp',
        default_value='true',
        description='Choose C++ controller (true) or Python controller (false)'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true (default: true)'
    )

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params,
        description='Path to controller_params.yaml'
    )

    # 1. C++ Controller Node
    cpp_controller_node = Node(
        package='mobile_robot_controller',
        executable='diff_drive_controller_cpp',
        name='diff_drive_controller',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        condition=IfCondition(LaunchConfiguration('use_cpp'))
    )

    # 2. Python Controller Node
    py_controller_node = Node(
        package='mobile_robot_controller',
        executable='diff_drive_controller_py',
        name='diff_drive_controller',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        condition=UnlessCondition(LaunchConfiguration('use_cpp'))
    )

    # 3. Spawners for ros2_control hardware interface
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager',
            '/controller_manager',
            '--unload-on-kill',
        ],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen'
    )

    joint_group_velocity_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_group_velocity_controller',
            '--controller-manager',
            '/controller_manager',
            '--unload-on-kill',
        ],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen'
    )

    return LaunchDescription([
        use_cpp_arg,
        use_sim_time_arg,
        params_file_arg,
        joint_state_broadcaster_spawner,
        joint_group_velocity_controller_spawner,
        cpp_controller_node,
        py_controller_node
    ])
