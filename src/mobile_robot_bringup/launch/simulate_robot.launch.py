#!/usr/bin/env python3
"""
================================================================================
Simulate Robot Bringup Launch File (Sequential Staged Startup)
--------------------------------------------------------------------------------
Author: Rahul Ramasamy
Email: rahul.r.joshua123@gmail.com
GitHub: https://github.com/rahul-r-joshua
Portfolio: https://rahul-r-joshua.github.io/portfolio/

Workshop Edition for ROS 2 Mobile Robotics Workshop.

Staged Timer Execution (prevents race conditions and RViz TF startup errors):
  1. T = 0.0s: Gazebo Physics & Robot Description (spawn robot on ground)
  2. T = 3.0s: Controller Spawner (ros2_control joint_state_broadcaster & diff_drive)
  3. T = 4.5s: Twist Mux & Joystick Teleop (/joy_vel, /cmd_vel_raw routing)
  4. T = 5.5s: Safety Zone Controller (LiDAR active collision avoidance)
  5. T = 6.5s: RViz2 (Visualizer starts with stable TF tree & active topics)
================================================================================
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    TimerAction,
    LogInfo
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    desc_pkg = get_package_share_directory('mobile_robot_description')
    ctrl_pkg = get_package_share_directory('mobile_robot_controller')
    bringup_pkg = get_package_share_directory('mobile_robot_bringup')

    rviz_config_path = os.path.join(desc_pkg, 'rviz', 'simulation.rviz')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    use_ros2_control_arg = DeclareLaunchArgument(
        'use_ros2_control',
        default_value='true',
        description='Use ros2_control diff_drive (true) or custom C++/Python kinematics node (false)'
    )

    use_twist_mux_arg = DeclareLaunchArgument(
        'use_twist_mux',
        default_value='true',
        description='Enable Twist Mux velocity multiplexer and safety stop lock'
    )

    gazebo_backend_arg = DeclareLaunchArgument(
        'gazebo_backend',
        default_value='classic',
        description='Simulator backend: "classic" (Gazebo 11), "ign" (Ignition Fortress), or "gz" (Modern Gazebo)'
    )

    use_cpp_arg = DeclareLaunchArgument(
        'use_cpp',
        default_value='true',
        description='Use C++ safety zone node (true) or Python safety zone script (false)'
    )

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz2 visualizer (set false for headless/low-resource mode)'
    )

    # -------------------------------------------------------------------------
    # STAGE 1 (T = 0.0s): Gazebo Physics Simulation & Robot Description
    # -------------------------------------------------------------------------
    gazebo_classic_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(desc_pkg, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'use_ros2_control': LaunchConfiguration('use_ros2_control')
        }.items(),
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('gazebo_backend'), "' == 'classic'"])
        )
    )

    gazebo_ign_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(desc_pkg, 'launch', 'ign_gazebo.launch.py')
        ),
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('gazebo_backend'), "' in ['ign', 'gz']"])
        )
    )

    # -------------------------------------------------------------------------
    # STAGE 2 (T = 3.0s): Controllers (ros2_control or Custom Kinematics)
    # -------------------------------------------------------------------------
    ros2_controller_spawner = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ctrl_pkg, 'launch', 'ros2_controller.launch.py')
        ),
        launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time')}.items(),
        condition=IfCondition(LaunchConfiguration('use_ros2_control'))
    )

    custom_controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ctrl_pkg, 'launch', 'controller.launch.py')
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'use_cpp': LaunchConfiguration('use_cpp')
        }.items(),
        condition=UnlessCondition(LaunchConfiguration('use_ros2_control'))
    )

    stage2_controllers = TimerAction(
        period=3.0,
        actions=[
            LogInfo(msg=">>> [STAGE 2] Spawning Controllers..."),
            ros2_controller_spawner,
            custom_controller_launch
        ]
    )

    # -------------------------------------------------------------------------
    # STAGE 3 (T = 4.5s): Twist Mux & Joystick Teleop
    # -------------------------------------------------------------------------
    joystick_teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ctrl_pkg, 'launch', 'joystick_teleop.launch.py')
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'cmd_vel_out': 'cmd_vel_raw'
        }.items(),
        condition=IfCondition(LaunchConfiguration('use_twist_mux'))
    )

    stage3_teleop = TimerAction(
        period=4.5,
        actions=[
            LogInfo(msg=">>> [STAGE 3] Launching Twist Mux & Teleop Stack..."),
            joystick_teleop_launch
        ]
    )

    # -------------------------------------------------------------------------
    # STAGE 4 (T = 5.5s): Safety Zone Controller (Active LiDAR Collision Avoidance)
    # -------------------------------------------------------------------------
    safety_zone_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_pkg, 'launch', 'safety_zone.launch.py')
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'use_cpp': LaunchConfiguration('use_cpp')
        }.items()
    )

    stage4_safety = TimerAction(
        period=5.5,
        actions=[
            LogInfo(msg=">>> [STAGE 4] Activating Safety Zone Collision Avoidance..."),
            safety_zone_launch
        ]
    )

    # -------------------------------------------------------------------------
    # STAGE 5 (T = 6.5s): RViz2 Visualization (Launched when TF is live)
    # -------------------------------------------------------------------------
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('use_rviz'))
    )

    stage5_rviz = TimerAction(
        period=6.5,
        actions=[
            LogInfo(msg=">>> [STAGE 5] Launching RViz2 Visualizer..."),
            rviz_node
        ]
    )

    return LaunchDescription([
        use_sim_time_arg,
        use_ros2_control_arg,
        use_twist_mux_arg,
        gazebo_backend_arg,
        use_cpp_arg,
        use_rviz_arg,
        gazebo_classic_launch,
        gazebo_ign_launch,
        stage2_controllers,
        stage3_teleop,
        stage4_safety,
        stage5_rviz
    ])
