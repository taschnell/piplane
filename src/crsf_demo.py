import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray
import random

class Int16MultiArrayPublisher(Node):
    def __init__(self):
        super().__init__('int16_multi_array_publisher')
        self.publisher_ = self.create_publisher(Int16MultiArray, 'crsf_channels_data', 10)
        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = Int16MultiArray()
        # Fill the array with random integers between 988 and 2500
        msg.data = [(random.randint(988,2400)) for _ in range(16)]
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    
    int16_multi_array_publisher = Int16MultiArrayPublisher()
    
    rclpy.spin(int16_multi_array_publisher)
    
    int16_multi_array_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
