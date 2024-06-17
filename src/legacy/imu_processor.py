import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import numpy as np

class ImuProcessor(Node):
    def __init__(self):
        super().__init__('imu_processor')
        self.subscription = self.create_subscription(
            Imu,
            '/imu_data',
            self.imu_callback,
            10
        )
        self.subscription  # prevent unused variable warning

    def imu_callback(self, msg):
        # Extract quaternion from the IMU message
        quat = msg.orientation
        quat_array = [quat.w, quat.x, quat.y, quat.z]

        # Calculate pitch, yaw, and roll
        pitch, yaw, roll = self.quaternion_to_euler(quat_array)

        self.get_logger().info(f'Pitch: {pitch:.2f}, Yaw: {yaw:.2f}, Roll: {roll:.2f}')

    def quaternion_to_euler(self, quat):
        w, x, y, z = quat

        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        # Roll (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        roll = np.arcsin(sinp)

        # Pitch (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        pitch = np.arctan2(sinr_cosp, cosr_cosp)

        # Convert radians to degrees
        yaw = np.degrees(yaw)
        roll = np.degrees(roll)
        pitch = np.degrees(pitch)


        return pitch, yaw, roll


def main(args=None):
    rclpy.init(args=args)
    node = ImuProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
