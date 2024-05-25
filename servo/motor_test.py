import board
import busio
import rclpy
from rclpy.node import Node
from adafruit_pca9685 import PCA9685
from std_msgs.msg import Int16MultiArray

class DShotSubscriber(Node):
    def __init__(self):
        super().__init__('dshot_subscriber')
        self.subscription = self.create_subscription(
            Int16MultiArray,
            'crsf_channels_data',
            self.listener_callback,
            10)
        # Initialize I2C bus
        i2c = busio.I2C(board.SCL, board.SDA)
        # Initialize the PCA9685 PWM controller with the I2C bus
        self.pca9685 = PCA9685(i2c)
        self.pca9685.frequency = 1000  # Set frequency to 1000 Hz for DShot
        # Define the PWM channels connected to the motors
        self.throttle_channel = 0  # Adjust the channel number as needed

    def listener_callback(self, msg):
        # Assuming that msg.data contains the channel data in the expected format
        throttle = msg.data[2]  # Assuming throttle data is in the first element
        # Adjusting the range from 988-2500 to 0-2047 for DShot signal
        dshot_value = int((throttle - 988) * (2047 / (2012 - 988)))
        print(dshot_value)
        # Send DShot signal to the motor
        self.pca9685.channels[self.throttle_channel].duty_cycle = dshot_value

def main(args=None):
    rclpy.init(args=args)

    dshot_subscriber = DShotSubscriber()

    rclpy.spin(dshot_subscriber)

    dshot_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
