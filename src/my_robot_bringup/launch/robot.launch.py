import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Get the path to your own package's launch directory
    my_robot_bringup_dir = get_package_share_directory('my_robot_bringup')

    # Path to the included hardware launch file
    hardware_launch_path = os.path.join(
        my_robot_bringup_dir,
        'launch',
        'bringup.launch.py' # This launches LiDAR and robot_state_publisher
    )
    
    # Path to the included sensor fusion launch file
    fusion_launch_path = os.path.join(
        my_robot_bringup_dir,
        'launch',
        'robot_localization.launch.py' # This launches the EKF
    )

    return LaunchDescription([
        # 1. Include the hardware launch file (LiDAR, robot_state_publisher)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(hardware_launch_path)
        ),

        # 2. Start the vehicle_driver node (motors and raw odometry)
        Node(
            package='my_robot_driver',
            executable='vehicle_driver',
            name='vehicle_driver_node',
            output='screen'
        ),
        
        # 3. Start the IMU publisher node (with auto-calibration)
        Node(
            package='my_robot_driver',
            executable='imu_publisher',
            name='imu_publisher_node',
            output='screen'
        ),
        
        # 4. Include the sensor fusion launch file (EKF)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(fusion_launch_path)
        ),
    ])
