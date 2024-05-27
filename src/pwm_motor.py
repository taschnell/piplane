"""
BASIC Pwm Signal to control an AC motov via the adafruit_pca9685 --> Still needs a throttle curve, currently linear.
"""

import board
import busio
import rclpy
from rclpy.node import Node
from adafruit_pca9685 import PCA9685
from std_msgs.msg import Int16MultiArray

class PWMSubscriber(Node):
    def __init__(self):
        super().__init__('pwm_subscriber')
        self.subscription = self.create_subscription(
            Int16MultiArray,
            'crsf_channels_data',
            self.listener_callback,
            10)
        # Initialize I2C bus
        i2c = busio.I2C(board.SCL, board.SDA)
        # Initialize the PCA9685 PWM controller with the I2C bus
        self.pca9685 = PCA9685(i2c)
        self.pca9685.frequency = 50  # Set frequency to 50 Hz for standard PWM ESC
        self.throttle_channel = 0 
        self.armed = False
    def listener_callback(self, msg):
        armed = msg.data[4] > 1000
        # print(msg.data[2], msg.data[4])
        throttle = msg.data[2] 
        # Adjusting the range from 988-2012 to 1000-2000 for PWM signal
        if throttle < 988:
            throttle = 988
        elif throttle > 2012:
            throttle = 2012
        pwm_value_us = (throttle - 988) * (1000 / (2012 - 988)) + 1000
        # Convert the pulse width to PCA9685 duty cycle
        
        duty_cycle = int(pwm_value_us * (65535 / 20000))  # 20 ms period
        # Send PWM signal to the motor
        if armed:
            self.pca9685.channels[self.throttle_channel].duty_cycle = duty_cycle
            print(duty_cycle, pwm_value_us, armed)

        else:
            # Throttle = 0 if not armed
            self.pca9685.channels[self.throttle_channel].duty_cycle = 3276
            print(3276, pwm_value_us, armed)
        
        armed = False


def main(args=None):
    rclpy.init(args=args)

    pwm_subscriber = PWMSubscriber()

    rclpy.spin(pwm_subscriber)

    pwm_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
