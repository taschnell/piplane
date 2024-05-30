import time
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu  # Import the Imu message

class ImuPublisher(Node):

    def __init__(self):
        super().__init__('imu_publisher')
        self.imu_pub = self.create_publisher(Imu, '/imu_data', 10)  # Topic: /imu_data, queue size: 10
        self.timer = self.create_timer(0.01, self.publish_data)  # Publish every 0.01 seconds

        # Replace with your I2C initialization code (assuming it's already done)
        # i2c = busio.I2C(board.SCL, board.SDA)
        # bno = BNO08X_I2C(i2c)
        # ... sensor initialization

    def publish_data(self):
        imu_msg = Imu()

        # Read sensor data from BNO08X
        accel_x, accel_y, accel_z = self.read_accel()  # Replace with your sensor reading function
        gyro_x, gyro_y, gyro_z = self.read_gyro()  # Replace with your sensor reading function
        # ... similar for quaternion

        # Fill the Imu message
        imu_msg.linear_acceleration.x = accel_x
        imu_msg.linear_acceleration.y = accel_y
        imu_msg.linear_acceleration.z = accel_z
        imu_msg.angular_velocity.x = gyro_x
        imu_msg.angular_velocity.y = gyro_y
        imu_msg.angular_velocity.z = gyro_z
        # ... similar for quaternion data (if using custom message)

        self.imu_pub.publish(imu_msg)

def main():
    rclpy.init()
    node = ImuPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
