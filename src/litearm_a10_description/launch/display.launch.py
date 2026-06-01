import os
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('litearm_a10_description').find('litearm_a10_description')
    urdf_path = os.path.join(pkg_share, 'urdf', 'LiteArm_A10_251224.urdf')

    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    set_env = SetEnvironmentVariable(
        name='ROS_PACKAGE_PATH',
        value=os.environ.get('ROS_PACKAGE_PATH', '') + ':' + pkg_share
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen'
    )

    rviz_config = os.path.join(pkg_share, 'config', 'rviz.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )

    return LaunchDescription([
        set_env,
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node
    ])
