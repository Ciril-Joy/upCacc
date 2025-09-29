from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the path to our package
    my_robot_bringup_dir = get_package_share_directory('my_robot_bringup')
    
    # Path to the local EKF launch file (fuses odom + imu)
    local_ekf_launch_path = PathJoinSubstitution(
        [my_robot_bringup_dir, 'launch', 'robot_localization.launch.py']
    )
    
    # Path to the NAVIC driver launch file
    navic_launch_path = PathJoinSubstitution(
        [my_robot_bringup_dir, 'launch', 'navic.launch.py']
    )
    
    # Path to the global EKF config file
    global_ekf_config_path = PathJoinSubstitution(
        [my_robot_bringup_dir, 'config', 'global_ekf.yaml']
    )

    return LaunchDescription([
        # 1. Launch the local EKF (odom -> base_link)
        IncludeLaunchDescription(PythonLaunchDescriptionSource(local_ekf_launch_path)),

        # 2. Launch the NAVIC driver
        IncludeLaunchDescription(PythonLaunchDescriptionSource(navic_launch_path)),
        
        # 3. Start the navsat_transform_node
        # This crucial node converts GPS lat/lon to the robot's map frame.
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform_node',
            output='screen',
            parameters=[{
                'yaw_offset': 1.5707963, # Adjust this based on your IMU orientation (pi/2)
                'magnetic_declination_radians': 0.0, # Set for your location if needed
                'broadcast_cartesian_transform': True,
                'publish_filtered_gps': True,
                'use_odometry_yaw': False, # Use IMU heading instead
                'wait_for_datum': False,
            }],
            # Remap the topics to match our system
            remappings=[
                ('/imu', '/imu/data'),
                ('/gps/fix', '/fix'),
                ('/odometry/filtered', '/odometry/filtered'), # Input from local EKF
                ('/odometry/gps', '/odometry/gps')        # Output for global EKF
            ]
        ),
        
        # 4. Start the global EKF (map -> odom)
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='global_ekf_node',
            output='screen',
            parameters=[global_ekf_config_path],
            # Remap the output topic to avoid conflicts
            remappings=[('/odometry/filtered', '/odometry/global')]
        ),
    ])