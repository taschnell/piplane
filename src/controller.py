from enum import IntEnum
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray

class ChannelMap(IntEnum):
    """
    Some Info on how the channels are used by the drone.
    Channels 12-15 are unused, however support could be added
    If setting up ELRS TX, note that OpenTX and EdgeTX start at 1 not zero
        IE: 0 --> 1, 1 --> 2, etc.
    """
    ROLL = 0 # Continuous | 988 - 2011
    PITCH = 1 # Continuous | 988 - 2011
    THROTTLE = 2 # Continuous | 988 - 2011
    YAW = 3 # Continuous | 988 - 2011
    ARM = 4 # Two Modes | 988, 2011
    MODE = 5 # Three Modes | 988, 1500, 2011
    RIGHT_SWITCH = 6 # Two Modes | 988, 2011
    RIGHT_SWITCH_2 = 7 # Three Modes | 988, 1500, 2011
    BUTTON_1 = 8 # OFF/ON | 988, 2011
    BUTTON_2 = 9 # OFF/ON | 988, 2011
    BUTTON_3 = 10 # OFF/ON | 988, 2011
    BUTTON_4 = 11 # OFF/ON | 988, 2011
    # Channels 12-15 UNUSED


class Throttle_Publisher(Node):

    def __init__(self):
        super().__init__('motor_array')
        self.publisher_ = self.create_publisher(Int16MultiArray, 'motor_array', 1)

        self.current_values = [0] * 4

        self.timer = self.create_timer(0.01, self.timer_callback)

        self.subscription = self.create_subscription(
            Int16MultiArray,
            'crsf_channels_data',
            self.crsf_callback,
            1
        )
        self.crsf_channels = [0] * 16
        self.ARMED = False

    def crsf_callback(self, msg):
        self.crsf_channels = msg.data

    def timer_callback(self):
        array = Int16MultiArray()

        # Will only ARM if throttle is zero
        if not self.ARMED and self.crsf_channels[ChannelMap.THROTTLE] == 988:
           self.ARMED = self.crsf_channels[ChannelMap.ARM] == 2011
        elif self.ARMED and self.crsf_channels[ChannelMap.ARM] == 988:
            self.ARMED = False
        
        if self.ARMED:
            throttle_value = self.crsf_channels[2]
            mapped_throttle_value = self.exponential_mapping(throttle_value)
            self.current_values = [mapped_throttle_value] * 4
            array.data = self.current_values
            self.publisher_.publish(array)
            self.get_logger().info(f'Publishing: {array.data}')
        
        else: 
            self.current_values = [0,0,0,0]
            array.data = self.current_values
            self.publisher_.publish(array)
            self.get_logger().info(f'Publishing: {array.data}, NOT ARMED')

    def exponential_mapping(self, value):
        normalized_value = (value - 988) / (2012 - 988)
        expo_value = normalized_value ** 2
        return int(expo_value * 2047)

def main(args=None):
    rclpy.init(args=args)
    node = Throttle_Publisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
