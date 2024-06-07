import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray

class IntArrayPublisher(Node):

    def __init__(self):
        super().__init__('motor_array')
        self.publisher_ = self.create_publisher(Int32MultiArray, 'motor_array', 10)

        # Get user input once during initialization
        self.array_data = []
        for i in range(4):
            while True:
                try:
                    value = int(input(f"Enter an integer between 0 and 2047 for motor {i}: "))
                    if 0 <= value <= 2047:
                        self.array_data.append(value)
                        break
                    else:
                        self.get_logger().info('Value must be between 0 and 2047. Try again.')
                except ValueError:
                    self.get_logger().info('Invalid input. Enter an integer.')

        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        array = Int32MultiArray()
        array.data = self.array_data

        self.publisher_.publish(array)
        self.get_logger().info(f'Publishing: {array.data}')

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
