import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import serial
import time
from transforms3d.euler import euler2quat
import numpy as np

class VehicleDriver(Node):
    def __init__(self):
        super().__init__('vehicle_driver_with_odometry')
        
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 57600)
        serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value
        try:
            self.arduino = serial.Serial(serial_port, baud_rate, timeout=0.1)
            self.get_logger().info(f"Successfully connected to Arduino on {serial_port}")
            time.sleep(2)
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to connect to Arduino: {e}")
            rclpy.shutdown()
            return

        self.subscription = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        self.x, self.y, self.th = 0.0, 0.0, 0.0
        self.vx, self.vth = 0.0, 0.0
        self.last_time = self.get_clock().now()
        self.last_cmd_vel_time = self.get_clock().now()

        self.odom_timer = self.create_timer(0.02, self.publish_odometry)
        self.watchdog_timer = self.create_timer(0.1, self.watchdog_callback)
        self.get_logger().info('Vehicle Driver Node with Odometry has been started.')
    
    def watchdog_callback(self):
        if (self.get_clock().now() - self.last_cmd_vel_time).nanoseconds / 1e9 > 0.5:
            self.vx = 0.0
            self.vth = 0.0

    def cmd_vel_callback(self, msg: Twist):
        self.last_cmd_vel_time = self.get_clock().now()
        self.vx = msg.linear.x
        self.vth = msg.angular.z

        max_lin_speed_mapping = 1.12 # YOUR CALIBRATED VALUE
        max_ang_speed_mapping = 1.5  # This value is not critical when using IMU fusion
        speed_pwm_max = 200
        angle_pwm_max = 255
        speed_pwm = int((self.vx / max_lin_speed_mapping) * speed_pwm_max)
        angle_pwm = int((self.vth / max_ang_speed_mapping) * angle_pwm_max)
        speed_pwm = max(min(speed_pwm, speed_pwm_max), -speed_pwm_max)
        angle_pwm = max(min(angle_pwm, angle_pwm_max), -angle_pwm_max)
        command = f"<{speed_pwm},{angle_pwm}>\n"
        self.arduino.write(command.encode('utf-8'))

    def publish_odometry(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        if dt > 1.0:
            self.last_time = current_time
            return
        
        delta_x = self.vx * math.cos(self.th) * dt
        delta_y = self.vx * math.sin(self.th) * dt
        delta_th = self.vth * dt
        self.x += delta_x
        self.y += delta_y
        self.th += delta_th

        q = euler2quat(0, 0, self.th)

        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.w = q[0]
        odom.pose.pose.orientation.x = q[1]
        odom.pose.pose.orientation.y = q[2]
        odom.pose.pose.orientation.z = q[3]
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.angular.z = self.vth

        # High uncertainty for everything except forward velocity and yaw velocity
        odom.pose.covariance = np.diag([0.1, 0.1, 999.9, 999.9, 999.9, 0.1]).flatten().tolist()
        odom.twist.covariance = np.diag([0.1, 999.9, 999.9, 999.9, 999.9, 0.1]).flatten().tolist()
        
        self.odom_pub.publish(odom)
        self.last_time = current_time

    def on_shutdown(self):
        self.get_logger().info("Shutting down node, stopping motors.")
        command = "<0,0>\n"
        self.arduino.write(command.encode('utf-8'))
        self.arduino.close()

def main(args=None):
    rclpy.init(args=args)
    node = VehicleDriver()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.on_shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()