import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    package_share = get_package_share_directory('mission_logic')
    default_config_file = os.path.join(package_share, 'config', 'mission_logic.yaml')

    config_file = LaunchConfiguration('config_file')
    state_estimation_topic = LaunchConfiguration('state_estimation_topic')
    magnetic_field_topic = LaunchConfiguration('magnetic_field_topic')
    goal_pose_topic = LaunchConfiguration('goal_pose_topic')
    speed_topic = LaunchConfiguration('speed_topic')

    declare_config_file = DeclareLaunchArgument(
        'config_file',
        default_value=default_config_file,
        description='YAML parameter file for mission_node.',
    )
    declare_state_estimation_topic = DeclareLaunchArgument(
        'state_estimation_topic',
        default_value='/state_estimation',
        description='Odometry topic provided by the robot dog.',
    )
    declare_magnetic_field_topic = DeclareLaunchArgument(
        'magnetic_field_topic',
        default_value='/magnetic_field',
        description='Real SensorMsg topic published by the receiver driver or bridge.',
    )
    declare_goal_pose_topic = DeclareLaunchArgument(
        'goal_pose_topic',
        default_value='/goal_pose',
        description='PoseStamped navigation goal topic consumed by the robot dog.',
    )
    declare_speed_topic = DeclareLaunchArgument(
        'speed_topic',
        default_value='/speed',
        description='Optional speed command topic consumed by the robot dog.',
    )

    mission_node = Node(
        package='mission_logic',
        executable='mission_node',
        name='mission_node',
        output='screen',
        parameters=[config_file],
        remappings=[
            ('/state_estimation', state_estimation_topic),
            ('/magnetic_field', magnetic_field_topic),
            ('/goal_pose', goal_pose_topic),
            ('/speed', speed_topic),
        ],
    )

    return LaunchDescription([
        declare_config_file,
        declare_state_estimation_topic,
        declare_magnetic_field_topic,
        declare_goal_pose_topic,
        declare_speed_topic,
        mission_node,
    ])
