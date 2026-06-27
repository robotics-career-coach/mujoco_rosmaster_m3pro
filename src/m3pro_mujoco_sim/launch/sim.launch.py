import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node


def generate_launch_description():
    description_pkg = get_package_share_directory('m3pro_description')
    sim_pkg = get_package_share_directory('m3pro_mujoco_sim')

    robot_description_file = os.path.join(
        description_pkg, 'urdf', 'm3pro.urdf'
    )
    with open(robot_description_file, 'r') as f:
        robot_description_content = f.read()

    mujoco_model_path = os.path.join(description_pkg, 'mjcf', 'm3pro.xml')
    controllers_file = os.path.join(sim_pkg, 'config', 'controllers.yaml')
    mujoco_plugins_file = os.path.join(sim_pkg, 'config', 'mujoco_plugins.yaml')

    mujoco_ros2_control_node = Node(
        package='mujoco_ros2_control',
        executable='mujoco_ros2_control',
        output='screen',
        parameters=[
            {'mujoco_model_path': mujoco_model_path},
            {'robot_description': robot_description_content},
            controllers_file,
            mujoco_plugins_file,
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

    imu_sensor_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['imu_sensor_broadcaster', '--controller-manager', '/controller_manager'],
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

    delay_imu = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[imu_sensor_broadcaster_spawner],
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

    return LaunchDescription([
        mujoco_ros2_control_node,
        robot_state_publisher_node,
        joint_state_broadcaster_spawner,
        delay_mecanum,
        delay_imu,
        delay_arm,
        delay_gripper,
    ])
