import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Declare arguments
    config_file = LaunchConfiguration('config_file')
    control_mode = LaunchConfiguration('control_mode')

    # Get package paths
    litearm_config_path = FindPackageShare('litearm_config')
    litearm_description_path = FindPackageShare('litearm_a10_251125')

    # Default robot config file
    default_config_file = PathJoinSubstitution([
        litearm_config_path,
        'robot_param',
        'litearm_right_arm.yaml'
    ])

    # Controller parameters file for real hardware
    controllers_file = PathJoinSubstitution([
        litearm_config_path,
        'config',
        'ros2_controllers_hardware.yaml'
    ])

    # URDF file path - hardware version
    urdf_file = PathJoinSubstitution([
        litearm_config_path,
        'config',
        'LiteArm_A10_251125_hardware.urdf.xacro'
    ])

    # Generate robot_description with hardware interface
    robot_description_content = ParameterValue(
        Command([
            FindExecutable(name='xacro'), ' ', urdf_file,
            ' config_file:=', config_file,
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
    # 2. Controller Manager
    # ============================================
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            controllers_file,
            {'robot_description': robot_description_content},
            {'use_sim_time': False},  # Must be LAST to override
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

    # Right Arm Controller Spawner (delayed start after joint_state_broadcaster)
    right_arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['right_arm_controller',
                   '--controller-manager-timeout', '60',
                   '--controller-manager', '/controller_manager'],
        output='screen'
    )

    # Right Gripper Controller Spawner (delayed start after right_arm_controller)
    right_gripper_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['right_gripper_controller',
                   '--controller-manager-timeout', '60',
                   '--controller-manager', '/controller_manager'],
        output='screen'
    )

    # Delay right arm controller start until joint_state_broadcaster is loaded
    delay_right_arm_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[right_arm_controller_spawner],
        )
    )

    # Delay right gripper controller start until right_arm_controller is loaded
    delay_right_gripper_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=right_arm_controller_spawner,
            on_exit=[right_gripper_controller_spawner],
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=default_config_file,
            description='Path to robot configuration YAML file'
        ),
        DeclareLaunchArgument(
            'control_mode',
            default_value='position_velocity',
            description='Control mode: position_velocity, pd_control, or full_control'
        ),
        robot_state_publisher,
        controller_manager,
        joint_state_broadcaster_spawner,
        delay_right_arm_controller_spawner,
        delay_right_gripper_controller_spawner,
    ])