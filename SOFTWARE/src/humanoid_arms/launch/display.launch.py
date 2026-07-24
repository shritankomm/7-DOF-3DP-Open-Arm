import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('humanoid_arms')
    urdf_path = os.path.join(pkg_share, 'urdf', 'humanoid_arms_fixed.urdf')

    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    rviz_config_path = os.path.join(pkg_share, 'rviz', 'display.rviz')
    rviz_args = ['-d', rviz_config_path] if os.path.exists(rviz_config_path) else []

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=rviz_args,
        ),
    ])
