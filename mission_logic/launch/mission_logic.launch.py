import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('mission_logic'),
        'config',
        'mission_logic.yaml',
    )
    pipeline_config_file = os.path.join(
        get_package_share_directory('mission_logic'),
        'config',
        'straight_wire.json',
    )

    use_sim_magnetic_field = LaunchConfiguration('use_sim_magnetic_field')
    start_mission_node_arg = LaunchConfiguration('start_mission_node')
    pipeline_config_file_arg = LaunchConfiguration('pipeline_config_file')

    declare_use_sim_magnetic_field = DeclareLaunchArgument(
        'use_sim_magnetic_field',
        default_value='true',
        description='Start the simulated magnetic field node.',
    )
    declare_start_mission_node = DeclareLaunchArgument(
        'start_mission_node',
        default_value='true',
        description='Start the mission decision node.',
    )
    declare_pipeline_config_file = DeclareLaunchArgument(
        'pipeline_config_file',
        default_value=pipeline_config_file,
        description='JSON pipeline configuration used by the simulated magnetic field node.',
    )

    start_magnetic_field_node = Node(
        package='mission_logic',
        executable='magnetic_field_node',
        name='magnetic_field_node',
        output='screen',
        condition=IfCondition(use_sim_magnetic_field),
        parameters=[
            config_file,
            {'pipeline_config_file': pipeline_config_file_arg},
        ],
    )

    start_mission_node = Node(
        package='mission_logic',
        executable='mission_node',
        name='mission_node',
        output='screen',
        condition=IfCondition(start_mission_node_arg),
        parameters=[config_file],
    )

    return LaunchDescription([
        declare_use_sim_magnetic_field,
        declare_start_mission_node,
        declare_pipeline_config_file,
        start_magnetic_field_node,
        start_mission_node,
    ])
