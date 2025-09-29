from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # This node starts the NMEA driver
        Node(
            package='nmea_navsat_driver',
            executable='nmea_serial_driver',
            name='navsat_driver',
            output='screen',
            parameters=[{
                # The Pi's primary hardware UART is usually /dev/ttyS0
                # If you use a USB-to-Serial adapter, it will be /dev/ttyUSBx
                'port': '/dev/navic',
                
                # Baud rate from your NAVIC module's datasheet
                'baud': 115200,
                
                # This frame_id MUST match the link name in your URDF
                'frame_id': 'gps_link'
            }]
        )
    ])