# ~/ros2_ws/src/my_robot_bringup/launch/imu.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """Launch the MPU6050 driver node."""
    
    mpu6050_driver_node = Node(
        package='mpu6050_driver_ros',
        executable='mpu6050_driver_node',
        name='mpu6050_driver_node',
        output='screen',
        parameters=[{
            'frame_id': 'imu_link', # We'll add this frame to the URDF later
            # 'i2c_bus_num': 1,     # Usually 1 for Raspberry Pi, default is often correct
            # 'device_addr': 104    # 0x68 in decimal, default is correct
        }]
    )

    return LaunchDescription([
        mpu6050_driver_node
    ])
