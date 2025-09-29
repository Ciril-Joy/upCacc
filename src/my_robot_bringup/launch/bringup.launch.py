import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    urdf_file_path = os.path.join(
        get_package_share_directory('my_robot_bringup'),
        'urdf',
        'my_robot.urdf')

    sllidar_launch_path = os.path.join(
        get_package_share_directory('sllidar_ros2'),
        'launch',
        'sllidar_a1_launch.py')

    robot_description_content = ParameterValue(
        Command(['xacro ', urdf_file_path]),
        value_type=str
    )

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description_content}]
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sllidar_launch_path)
        )
    ])
