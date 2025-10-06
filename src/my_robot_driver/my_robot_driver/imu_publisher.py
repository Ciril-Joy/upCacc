import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import board
import busio
import adafruit_mpu6050
import time
import numpy as np

class IMUPublisher(Node):
    def __init__(self):
        super().__init__('imu_publisher')
        
        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('frame_id', 'imu_link')
        
        topic = self.get_parameter('imu_topic').get_parameter_value().string_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value

        self.publisher_ = self.create_publisher(Imu, topic, 10)
        
        # --- Bias and Filter variables ---
        self.gyro_x_bias, self.gyro_y_bias, self.gyro_z_bias = 0.0, 0.0, 0.0
        self.accel_x_bias, self.accel_y_bias, self.accel_z_bias = 0.0, 0.0, 0.0
        
        # Low-pass filter variables for accelerometer
        self.alpha = 0.1 # Smoothing factor. Lower value = more smoothing.
        self.last_accel_x, self.last_accel_y, self.last_accel_z = 0.0, 0.0, 0.0

        try:
            i2c = busio.I2C(board.SCL, board.SDA)
            self.sensor = adafruit_mpu6050.MPU6050(i2c)
            self.get_logger().info('Adafruit MPU6050 Initialized Successfully')
            
            # --- Perform Full Calibration at Startup ---
            self.calibrate_sensors()

            # Start the main timer AFTER calibration is done
            self.timer = self.create_timer(0.02, self.timer_callback) # 50Hz

        except Exception as e:
            self.get_logger().error(f'Failed to initialize Adafruit MPU6050: {e}')
            rclpy.shutdown()
            return

    def calibrate_sensors(self):
        self.get_logger().info('Calibrating IMU... Do not move the robot. Ensure it is on a level surface.')
        num_samples = 1000
        
        # --- Gyro Calibration ---
        sum_gx, sum_gy, sum_gz = 0.0, 0.0, 0.0
        # --- Accel Calibration ---
        sum_ax, sum_ay, sum_az = 0.0, 0.0, 0.0
        
        for _ in range(num_samples):
            gyro_x, gyro_y, gyro_z = self.sensor.gyro
            accel_x, accel_y, accel_z = self.sensor.acceleration
            sum_gx += gyro_x
            sum_gy += gyro_y
            sum_gz += gyro_z
            sum_ax += accel_x
            sum_ay += accel_y
            sum_az += accel_z
            time.sleep(0.01)
            
        self.gyro_x_bias = sum_gx / num_samples
        self.gyro_y_bias = sum_gy / num_samples
        self.gyro_z_bias = sum_gz / num_samples
        
        self.accel_x_bias = sum_ax / num_samples
        self.accel_y_bias = sum_ay / num_samples
        # Z-axis bias needs to account for gravity (approx 9.8 m/s^2)
        self.accel_z_bias = (sum_az / num_samples) - 9.8 

        # Initialize the low-pass filter with the first calibrated reading
        self.last_accel_x = self.accel_x_bias
        self.last_accel_y = self.accel_y_bias
        self.last_accel_z = self.accel_z_bias
        
        self.get_logger().info('IMU calibration complete.')
        self.get_logger().info(f'  Gyro Bias (X, Y, Z): {self.gyro_x_bias:.4f}, {self.gyro_y_bias:.4f}, {self.gyro_z_bias:.4f}')
        self.get_logger().info(f'  Accel Bias (X, Y, Z): {self.accel_x_bias:.4f}, {self.accel_y_bias:.4f}, {self.accel_z_bias:.4f}')

    def timer_callback(self):
        try:
            msg = Imu()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.frame_id

            gyro_x_raw, gyro_y_raw, gyro_z_raw = self.sensor.gyro
            accel_x_raw, accel_y_raw, accel_z_raw = self.sensor.acceleration

            # --- Apply Gyro Calibration ---
            msg.angular_velocity.x = gyro_x_raw - self.gyro_x_bias
            msg.angular_velocity.y = gyro_y_raw - self.gyro_y_bias
            msg.angular_velocity.z = gyro_z_raw - self.gyro_z_bias * -1.0

            # --- Apply Accel Calibration ---
            accel_x_cal = accel_x_raw - self.accel_x_bias
            accel_y_cal = accel_y_raw - self.accel_y_bias
            accel_z_cal = accel_z_raw - self.accel_z_bias
            
            # --- Apply Low-Pass Filter ---
            # filtered_value = alpha * new_value + (1 - alpha) * last_filtered_value
            filtered_ax = self.alpha * accel_x_cal + (1 - self.alpha) * self.last_accel_x
            filtered_ay = self.alpha * accel_y_cal + (1 - self.alpha) * self.last_accel_y
            filtered_az = self.alpha * accel_z_cal + (1 - self.alpha) * self.last_accel_z
            
            # Update last values for next iteration
            self.last_accel_x = filtered_ax
            self.last_accel_y = filtered_ay
            self.last_accel_z = filtered_az
            
            msg.linear_acceleration.x = filtered_ax
            msg.linear_acceleration.y = filtered_ay
            msg.linear_acceleration.z = filtered_az
            
            # --- Covariances ---
            msg.orientation_covariance[0] = -1.0
            msg.angular_velocity_covariance = np.diag([0.001, 0.001, 0.001]).flatten().tolist()
            # We are still uncertain, but less so than raw odometry
            msg.linear_acceleration_covariance = np.diag([0.05, 0.05, 0.05]).flatten().tolist()
            
            self.publisher_.publish(msg)

        except Exception as e:
            self.get_logger().warn(f'Could not read from MPU6050: {e}')

def main(args=None):
    rclpy.init(args=args)
    imu_publisher = IMUPublisher()
    rclpy.spin(imu_publisher)
    imu_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
