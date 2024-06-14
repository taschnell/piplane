import rclpy
from rclpy.node import Node
import board
import busio

from sensor_msgs.msg import Imu  # Import the Imu message
from adafruit_bno08x import (
    BNO_REPORT_ACCELEROMETER,
    BNO_REPORT_GYROSCOPE,
    BNO_REPORT_MAGNETOMETER,
    BNO_REPORT_ROTATION_VECTOR,
)
from adafruit_bno08x.i2c import BNO08X_I2C


class ImuPublisher(Node):

    def __init__(self):
        super().__init__('imu_publisher')
        self.get_logger().info('Initializing IMU Publisher Node')
        
        self.imu_pub = self.create_publisher(Imu, '/imu_data', 1)  # Topic: /imu_data, queue size: 10
        self.timer = self.create_timer(0.01, self.publish_data)  # Publish every 0.01 seconds

        i2c = busio.I2C(board.SCL, board.SDA)
        self.bno = BNO08X_I2C(i2c)

        self.get_logger().info('Enabling BNO08X features')
        self.bno.enable_feature(BNO_REPORT_ACCELEROMETER)
        self.bno.enable_feature(BNO_REPORT_GYROSCOPE)
        # self.bno.enable_feature(BNO_REPORT_MAGNETOMETER)
        self.bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)

    def publish_data(self):
        imu_msg = Imu()

        accel_x, accel_y, accel_z = self.bno.acceleration 
        gyro_x, gyro_y, gyro_z = self.bno.gyro 
        quat_i, quat_j, quat_k, quat_real = self.bno.quaternion

        imu_msg.linear_acceleration.x = accel_x
        imu_msg.linear_acceleration.y = accel_y
        imu_msg.linear_acceleration.z = accel_z # Positive is down
        imu_msg.angular_velocity.x = gyro_x
        imu_msg.angular_velocity.y = gyro_y
        imu_msg.angular_velocity.z = gyro_z
        imu_msg.orientation.w = quat_real
        imu_msg.orientation.x = quat_i
        imu_msg.orientation.y = quat_j
        imu_msg.orientation.z = quat_k

        self.get_logger().info(f'Publishing IMU data: Accel=({accel_x}, {accel_y}, {accel_z})')
        self.imu_pub.publish(imu_msg)

def main():
    rclpy.init()
    node = ImuPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
