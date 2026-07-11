import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    sim_pkg = get_package_share_directory('m3pro_mujoco_sim')
    description_pkg = get_package_share_directory('m3pro_description')

    scene_arg = DeclareLaunchArgument(
        'scene', default_value='empty',
        description='Scene to load (maps to mjcf/scene_<value>.xml)',
    )

    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sim_pkg, 'launch', 'sim.launch.py')
        ),
        launch_arguments={'scene': LaunchConfiguration('scene')}.items(),
    )

    rviz_config = os.path.join(description_pkg, 'rviz', 'm3pro.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    return LaunchDescription([
        scene_arg,
        sim_launch,
        rviz_node,
    ])
