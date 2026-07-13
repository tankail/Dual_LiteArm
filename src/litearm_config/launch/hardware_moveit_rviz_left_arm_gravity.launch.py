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

    default_config_file = PathJoinSubstitution([
        litearm_config_path,
        'robot_param',
        'litearm_left_arm_ttyACM0.yaml'
    ])

    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config_file,
        description='Path to left arm robot configuration YAML'
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

    hardware_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                litearm_config_path,
                'launch',
                'hardware_left_arm.launch.py'
            ])
        ]),
        launch_arguments={
            'config_file': LaunchConfiguration('config_file'),
            'control_mode': LaunchConfiguration('control_mode'),
        }.items()
    )

    static_tfs_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                litearm_config_path,
                'launch',
                'static_virtual_joint_tfs.launch.py'
            ])
        ])
    )

    moveit_config = (
        MoveItConfigsBuilder("LiteArm_A10_251125", package_name="litearm_config")
        .trajectory_execution(file_path='config/moveit_controllers_hardware_left_arm.yaml')
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

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            move_group_configuration,
            {"use_sim_time": False},
        ],
    )

    rviz_config_file = PathJoinSubstitution([
        litearm_config_path,
        'config',
        'moveit.rviz'
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
        config_file_arg,
        control_mode_arg,
        rviz_arg,
        hardware_launch,
        static_tfs_launch,
        move_group_node,
        rviz_node,
    ])
