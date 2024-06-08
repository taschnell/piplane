"""
SETUP TO RUN PICO:
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 baudrate=115200
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray

class IntArrayPublisher(Node):

    def __init__(self):
        super().__init__('motor_array')
        self.publisher_ = self.create_publisher(Int16MultiArray, 'motor_array', 10)

        self.current_values = [0] * 4

        self.timer = self.create_timer(0.001, self.timer_callback)

        self.subscription = self.create_subscription(
            Int16MultiArray,
            'crsf_channels_data',
            self.crsf_callback,
            1
        )
        self.crsf_channels = [0] * 16

    def crsf_callback(self, msg):
        self.crsf_channels = msg.data

    def timer_callback(self):
        array = Int16MultiArray()

        throttle_value = self.crsf_channels[2]

        mapped_throttle_value = self.exponential_mapping(throttle_value)

        self.current_values = [mapped_throttle_value] * 4

        array.data = self.current_values
        self.publisher_.publish(array)
        self.get_logger().info(f'Publishing: {array.data}')

    def exponential_mapping(self, value):
        normalized_value = (value - 988) / (2012 - 988)
        expo_value = normalized_value ** 2
        return int(expo_value * 2047)

def main(args=None):
    rclpy.init(args=args)
    node = IntArrayPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
