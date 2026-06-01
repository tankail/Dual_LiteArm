import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    left_config_file = LaunchConfiguration('left_config_file')
    right_config_file = LaunchConfiguration('right_config_file')
    control_mode = LaunchConfiguration('control_mode')

    litearm_config_path = FindPackageShare('litearm_config')

    # Default robot config files
    default_left_config_file = PathJoinSubstitution([
        litearm_config_path, 'robot_param', 'litearm_left_arm.yaml'
    ])
    default_right_config_file = PathJoinSubstitution([
        litearm_config_path, 'robot_param', 'litearm_right_arm.yaml'
    ])

    # Combined controllers file
    controllers_file = PathJoinSubstitution([
        litearm_config_path, 'config', 'ros2_controllers_hardware_dual.yaml'
    ])

    # Dual-arm URDF file
    urdf_file = PathJoinSubstitution([
        litearm_config_path, 'config', 'LiteArm_A10_251125_hardware_dual.urdf.xacro'
    ])

    # Generate robot_description with both hardware interfaces
    robot_description_content = ParameterValue(
        Command([
            FindExecutable(name='xacro'), ' ', urdf_file,
            ' left_config_file:=', left_config_file,
            ' right_config_file:=', right_config_file,
            ' control_mode:=', control_mode
        ]),
        value_type=str
    )

    # ============================================
    # 1. Robot State Publisher
    # ============================================
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': False
        }]
    )

    # ============================================
    # 2. Controller Manager (manages both arms)
    # ============================================
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            controllers_file,
            {'robot_description': robot_description_content},
            {'use_sim_time': False},
        ],
        output='screen',
        arguments=['--ros-args', '--param', 'use_sim_time:=false'],
    )

    # ============================================
    # 3. Controller Spawners (sequential startup)
    # ============================================

    # Joint State Broadcaster Spawner
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster',
                   '--controller-manager-timeout', '60',
                   '--controller-manager', '/controller_manager'],
        output='screen'
    )

    # Left Arm Controller Spawner
    left_arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['left_arm_controller',
                   '--controller-manager-timeout', '60',
                   '--controller-manager', '/controller_manager'],
        output='screen'
    )

    # Left Gripper Controller Spawner
    left_gripper_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['left_gripper_controller',
                   '--controller-manager-timeout', '60',
                   '--controller-manager', '/controller_manager'],
        output='screen'
    )

    # Right Arm Controller Spawner
    right_arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['right_arm_controller',
                   '--controller-manager-timeout', '60',
                   '--controller-manager', '/controller_manager'],
        output='screen'
    )

    # Right Gripper Controller Spawner
    right_gripper_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['right_gripper_controller',
                   '--controller-manager-timeout', '60',
                   '--controller-manager', '/controller_manager'],
        output='screen'
    )

    # Sequential startup chain:
    # JS broadcaster -> left arm controller -> left gripper controller
    # JS broadcaster -> right arm controller -> right gripper controller
    # (left and right arms start in parallel after JS broadcaster)

    delay_left_arm_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[left_arm_controller_spawner],
        )
    )

    delay_left_gripper_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=left_arm_controller_spawner,
            on_exit=[left_gripper_controller_spawner],
        )
    )

    delay_right_arm_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[right_arm_controller_spawner],
        )
    )

    delay_right_gripper_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=right_arm_controller_spawner,
            on_exit=[right_gripper_controller_spawner],
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'left_config_file',
            default_value=default_left_config_file,
            description='Path to left arm robot configuration YAML'
        ),
        DeclareLaunchArgument(
            'right_config_file',
            default_value=default_right_config_file,
            description='Path to right arm robot configuration YAML'
        ),
        DeclareLaunchArgument(
            'control_mode',
            default_value='position_velocity',
            description='Control mode: position_velocity, pd_control, or full_control'
        ),
        robot_state_publisher,
        controller_manager,
        joint_state_broadcaster_spawner,
        delay_left_arm_controller,
        delay_left_gripper_controller,
        delay_right_arm_controller,
        delay_right_gripper_controller,
    ])
