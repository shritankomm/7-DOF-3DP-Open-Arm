import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
def generate_launch_description():
    pkg_share = get_package_share_directory('humanoid_arms')
    urdf_path = os.path.join(pkg_share, 'urdf', 'humanoid_arms_final.urdf')
    controllers_path = os.path.join(pkg_share, 'config', 'controllers.yaml')
    with open(urdf_path, 'r') as f:
        robot_description = f.read()
    robot_description = robot_description.replace('CONTROLLERS_PATH', controllers_path)
    # Point Gazebo at the meshes
    install_share_dir = os.path.dirname(pkg_share)
    os.environ['GZ_SIM_RESOURCE_PATH'] = (
        os.environ.get('GZ_SIM_RESOURCE_PATH', '') + ':' + install_share_dir
    )
    os.environ['GZ_SIM_SYSTEM_PLUGIN_PATH'] = '/opt/ros/jazzy/lib'
    # Launch Gazebo via the official include (waits for server to be ready)
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'),
                         'launch', 'gz_sim.launch.py')
        ]),
        launch_arguments={'gz_args': '-r empty.sdf'}.items(),
    )
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description,
                     'use_sim_time': True}],
    )
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        parameters=[{'lazy': False}],
        output='screen',
    )
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'humanoid_arms', '-topic', 'robot_description'],
        output='screen',
    )
    load_jsb = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster',
                   '--controller-manager-timeout', '60'],
        output='screen',
    )
    load_left_arm_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['left_arm_controller',
                   '--controller-manager-timeout', '60'],
        output='screen',
    )
    load_right_arm_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['right_arm_controller',
                   '--controller-manager-timeout', '60'],
        output='screen',
    )
    # Wait for the sim clock to be flowing before activating jsb (avoids the
    # 5-second activation timeout race). Delay the spawner a few seconds
    # after the robot spawns.
    delayed_jsb = TimerAction(period=5.0, actions=[load_jsb])
    return LaunchDescription([
        gz_sim,
        clock_bridge,
        robot_state_publisher,
        spawn,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn,
                on_exit=[delayed_jsb],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_jsb,
                on_exit=[load_left_arm_controller, load_right_arm_controller],
            )
        ),
    ])
