#! /home/taschnell/piplane/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray
from adafruit_servokit import ServoKit

class ServoControlNode(Node):
    def __init__(self):
        super().__init__('servo_control_node')
        self.subscription = self.create_subscription(
            Int16MultiArray,
            'crsf_channels_data',
            self.listener_callback,
            1
        )
        self.subscription  # prevent unused variable warning
        self.kit = ServoKit(channels=16)
        self.kit.servo[0].set_pulse_width_range(500, 2500)
        self.kit.servo[1].set_pulse_width_range(500, 2500)



    def listener_callback(self, msg):
        if len(msg.data) > 0:
            # Assuming channel 0 controls the servo
            channel_value = msg.data[0]
            channel_value = msg.data[1]

            # Map channel_value from 0-2500 to 0-180 degrees
            angle = self.map_value(channel_value, 988, 2012, 0, 180)
            self.kit.servo[0].angle = angle

            # self.get_logger().info(f'Servo angle set to: {angle} degrees |', end="")

            angle = self.map_value(channel_value, 988, 2012, 0, 180)
            self.kit.servo[1].angle = angle

            self.get_logger().info(f' Servo angle set to: {angle} degrees')

    @staticmethod
    def map_value(value, from_low, from_high, to_low, to_high):
        # Map the value from one range to another
        return to_low + (value - from_low) * (to_high - to_low) / (from_high - from_low)

def main(args=None):
    rclpy.init(args=args)
    servo_control_node = ServoControlNode()

    try:
        rclpy.spin(servo_control_node)
    except KeyboardInterrupt:
        pass
    finally:
        servo_control_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()