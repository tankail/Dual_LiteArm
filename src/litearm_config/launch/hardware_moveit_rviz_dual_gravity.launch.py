import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    litearm_config_path = FindPackageShare('litearm_config')

    default_left_config_file = PathJoinSubstitution([
        litearm_config_path, 'robot_param', 'litearm_left_arm.yaml'
    ])
    default_right_config_file = PathJoinSubstitution([
        litearm_config_path, 'robot_param', 'litearm_right_arm.yaml'
    ])

    left_config_file_arg = DeclareLaunchArgument(
        'left_config_file',
        default_value=default_left_config_file,
        description='Path to left arm robot configuration YAML'
    )

    right_config_file_arg = DeclareLaunchArgument(
        'right_config_file',
        default_value=default_right_config_file,
        description='Path to right arm robot configuration YAML'
    )

    control_mode_arg = DeclareLaunchArgument(
        'control_mode',
        default_value='full_control',
        description='Control mode: full_control (MIT mode with gravity compensation)'
    )

    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Start RViz'
    )

    # ============================================
    # 1. Dual Hardware Launch (both arms)
    # ============================================
    hardware_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                litearm_config_path, 'launch', 'hardware_dual.launch.py'
            ])
        ]),
        launch_arguments={
            'left_config_file': LaunchConfiguration('left_config_file'),
            'right_config_file': LaunchConfiguration('right_config_file'),
            'control_mode': LaunchConfiguration('control_mode'),
        }.items()
    )

    # ============================================
    # 2. Static TF (world -> base_link)
    # ============================================
    static_tfs_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                litearm_config_path, 'launch', 'static_virtual_joint_tfs.launch.py'
            ])
        ])
    )

    # ============================================
    # 3. MoveIt Config (both arms)
    # ============================================
    moveit_controllers_file = 'config/moveit_controllers_hardware_dual.yaml'

    moveit_config = (
        MoveItConfigsBuilder("LiteArm_A10_251125", package_name="litearm_config")
        .trajectory_execution(file_path=moveit_controllers_file)
        .to_moveit_configs()
    )

    move_group_configuration = {
        "publish_robot_description_semantic": True,
        "allow_trajectory_execution": True,
        "publish_robot_description": True,
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
        "monitor_dynamics": False,
        "use_sim_time": False,
    }

    move_group_params = [
        moveit_config.to_dict(),
        move_group_configuration,
        {"use_sim_time": False},
    ]

    # ============================================
    # 4. Move Group Node
    # ============================================
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=move_group_params,
    )

    # ============================================
    # 5. RViz Node
    # ============================================
    rviz_config_file = PathJoinSubstitution([
        litearm_config_path, 'config', 'moveit.rviz'
    ])

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": False},
        ],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    return LaunchDescription([
        left_config_file_arg,
        right_config_file_arg,
        control_mode_arg,
        rviz_arg,
        hardware_launch,
        static_tfs_launch,
        move_group_node,
        rviz_node,
    ])
