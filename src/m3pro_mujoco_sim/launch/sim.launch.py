import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, Shutdown, TimerAction
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    description_pkg = get_package_share_directory('m3pro_description')
    sim_pkg = get_package_share_directory('m3pro_mujoco_sim')

    scene_arg = DeclareLaunchArgument(
        'scene', default_value='empty',
        description='Scene to load (maps to mjcf/scene_<value>.xml)',
    )

    robot_description_file = os.path.join(
        description_pkg, 'urdf', 'm3pro.urdf'
    )
    with open(robot_description_file, 'r') as f:
        robot_description_content = f.read()

    mjcf_dir = os.path.join(description_pkg, 'mjcf', '')
    controllers_file = os.path.join(sim_pkg, 'config', 'controllers.yaml')
    mujoco_plugins_file = os.path.join(sim_pkg, 'config', 'mujoco_plugins.yaml')

    # Publishes MJCF XML to /mujoco_robot_description (latching) for ros2_control_node
    mjcf_publisher_node = Node(
        package='m3pro_description',
        executable='mjcf_publisher',
        name='mjcf_description_publisher',
        parameters=[{
            'mjcf_path': [mjcf_dir, 'scene_', LaunchConfiguration('scene'), '.xml'],
        }],
        output='screen',
    )

    mujoco_ros2_control_node = Node(
        package='mujoco_ros2_control',
        executable='ros2_control_node',
        output='screen',
        parameters=[
            controllers_file,
            mujoco_plugins_file,
        ],
        remappings=[
            ('~/robot_description', '/robot_description'),
            ('/mecanum_drive_controller/tf_odometry', '/tf'),
        ],
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': robot_description_content},
            {'use_sim_time': True},
        ],
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    mecanum_drive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['mecanum_drive_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    gripper_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['gripper_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    # Chain controller spawning: joint_state_broadcaster first, then the rest
    delay_mecanum = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[mecanum_drive_controller_spawner],
        )
    )

    delay_arm = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[arm_controller_spawner],
        )
    )

    delay_gripper = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[gripper_controller_spawner],
        )
    )

    shutdown_on_sim_exit = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=mujoco_ros2_control_node,
            on_exit=[Shutdown(reason='MuJoCo simulation exited')],
        )
    )

    # Start ros2_control_node 1 second after mjcf_publisher to ensure topic is available
    delay_ros2_control = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=mjcf_publisher_node,
            on_start=[
                TimerAction(
                    period=1.0,
                    actions=[mujoco_ros2_control_node],
                )
            ],
        )
    )

    return LaunchDescription([
        scene_arg,
        mjcf_publisher_node,
        delay_ros2_control,
        shutdown_on_sim_exit,
        robot_state_publisher_node,
        joint_state_broadcaster_spawner,
        delay_mecanum,
        delay_arm,
        delay_gripper,
    ])
