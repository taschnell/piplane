from enum import IntEnum
import rclpy
import numpy as np
import math
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray
from sensor_msgs.msg import Imu

"""
SETUP TO RUN PICO:
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 baudrate=115200
"""

"""
Some Info on how the channels are used by the drone.
Channels 12-15 are unused, however support could be added
If setting up ELRS TX, note that OpenTX and EdgeTX start at 1 not zero
    IE: 0 --> 1, 1 --> 2, etc.
"""
# MID IS 1500
ROLL = 0  # Continuous | 988 - 2011
PITCH = 1  # Continuous | 988 - 2011
THROTTLE = 2  # Continuous | 988 - 2011
YAW = 3  # Continuous | 988 - 2011
ARM = 4  # Two Modes | 988, 2011 | FALSE, TRUE
MODE = 5  # Three Modes | 988, 1500, 2011 | "ANGLE", TBD, "ACRO"
RIGHT_SWITCH = 6  # Two Modes | 988, 2011
RIGHT_SWITCH_2 = 7  # Three Modes | 988, 1500, 2011
BUTTON_1 = 8  # OFF/ON | 988, 2011
BUTTON_2 = 9  # OFF/ON | 988, 2011
BUTTON_3 = 10  # OFF/ON | 988, 2011
BUTTON_4 = 11  # OFF/ON | 988, 2011
# Channels 12-15 UNUSED

EXPO = 2

class MotorMap(IntEnum):
    # Isn't it great how hardware goes from 1-4, but Programming indexs are from 0-3
    MOTOR_1 = 0
    MOTOR_2 = 1
    MOTOR_3 = 2
    MOTOR_4 = 3

class Throttle_Publisher(Node):

    def __init__(self):
        super().__init__("motor_array")
        self.publisher_dshot = self.create_publisher(Int16MultiArray, "motor_array", 1)

        self.current_values = [0] * 4

        self.subscription = self.create_subscription(
            Int16MultiArray, "crsf_channels_data", self.crsf_callback, 1
        )
        self.imu_subscription = self.create_subscription(
            Imu, "imu_data", self.imu_callback, 1
        )

        self.timer = self.create_timer(0.01, self.timer_callback)

        self.crsf_channels = [0] * 16
        self.ARMED = False
        self.MODE = "ANGLE"  # Can also Be ACRO
        self.euler_orientation = None
        self.linear_acc = None
        self.angular_vel = None
        self.g_force = 0

    def crsf_callback(self, msg):
        self.crsf_channels = msg.data

    def imu_callback(self, msg):
        # Handling IMU data
        # TODO Stablization Code
        self.linear_acc = msg.linear_acceleration
        self.g_force = round(
            math.sqrt(
                self.linear_acc.x**2 + self.linear_acc.y**2 + self.linear_acc.z**2
            )
            / 9.81,
            2,
        )

        self.angular_vel = msg.angular_velocity
        quat = msg.orientation
        self.euler_orientation = self.quaternion_to_euler(
            [quat.w, quat.x, quat.y, quat.z]
        )
        # self.get_logger().info(f"{self.g_force}"
        # )

    def timer_callback(self):
        array = Int16MultiArray()

        # Will only ARM if throttle is zero
        if not self.ARMED and self.crsf_channels[THROTTLE] == 988:
            self.ARMED = self.crsf_channels[ARM] == 2011
        elif self.ARMED and self.crsf_channels[ARM] == 988:
            self.ARMED = False

        # Flight Mode, add automated mode later?
        if self.crsf_channels[MODE] < 1500:
            self.MODE = "ANGLE"
        elif self.crsf_channels[MODE] > 1500:
            self.MODE = "ACRO"

        if self.ARMED:
            throttle_value = self.crsf_channels[THROTTLE]
            mapped_throttle_value = self.exponential_mapping(throttle_value, EXPO)

            self.current_values = [mapped_throttle_value] * 4

            # Angle or Acro Mode, maybe add others later.
            if self.MODE == "ACRO":
                self.acro_throttle_transformation()
            elif self.MODE == "ANGLE":
                self.angle_throttle_transformation()

            for i in range(4):
                # Min Value of 100, Max of 2047 when armed
                if self.current_values[i] > 2047:
                    self.current_values[i] = 2047
                elif self.current_values[i] < 100:
                    self.current_values[i] = 100

            array.data = self.current_values

        else:
            # Motors set to zero if not armed
            self.current_values = [0, 0, 0, 0]
            array.data = self.current_values

        self.publisher_dshot.publish(array)
        self.get_logger().info(
            f"Publishing: {array.data} | ARMED: {self.ARMED} | MODE: {self.MODE}"
        )

    def exponential_mapping(self, value, expo=2):
        # Isn't math cool!?
        normalized_value = (value - 988) / (2012 - 988)
        expo_value = normalized_value**expo
        return int(expo_value * 1947 + 100)

    def angle_throttle_transformation(self):
        """
        Angle Mode, changes drone's angle by a maximum of 25 degrees relative to level, in pitch and roll direction, also handles yaw (same as acro)

        Think of this as an auto stabilizer
        
        These controls might end up inverted depending on IMU Position, make sure to test that

        Should adjust motor speed less drastically if lower error value, and higher if a greater error value
        """
        DEGREES = 25
        MAX_THROTTLE_ADJUSTMENT = (
            15  # Adjust this value to control sensitivity, max throttle change is 15%
        )

        RATE = 100

        roll_angle = self.percent_map(self.crsf_channels[ROLL], EXPO, RATE) * DEGREES
        pitch_angle = self.percent_map(self.crsf_channels[PITCH], EXPO, RATE) * DEGREES
        yaw_per = self.percent_map(self.crsf_channels[YAW], EXPO, RATE)

        imu_pitch = self.euler_orientation[0]
        # imu_yaw = self.euler_orientation[1]
        imu_roll = self.euler_orientation[2]

        roll_err = roll_angle - imu_roll
        pitch_err = pitch_angle - imu_pitch

        # self.get_logger().info(
        #     f"{roll_err}: {self.angle_expo_adj_per(roll_err, rate=MAX_THROTTLE_ADJUSTMENT)}, {pitch_err}: {self.angle_expo_adj_per(pitch_err, rate=MAX_THROTTLE_ADJUSTMENT)}"
        # )

        roll_per = self.angle_expo_adj_per(roll_err, EXPO, MAX_THROTTLE_ADJUSTMENT)
        pitch_per = self.angle_expo_adj_per(pitch_err, EXPO, MAX_THROTTLE_ADJUSTMENT)

        # Note this works because all throttle values are initially the same
        roll_vals = round(self.current_values[MotorMap.MOTOR_1] * roll_per)
        pitch_vals = round(self.current_values[MotorMap.MOTOR_1] * pitch_per)
        yaw_vals = round(self.current_values[MotorMap.MOTOR_1] * yaw_per)

        self.current_values[MotorMap.MOTOR_1] = (
            self.current_values[MotorMap.MOTOR_1] - roll_vals + pitch_vals + yaw_vals
        )
        self.current_values[MotorMap.MOTOR_2] = (
            self.current_values[MotorMap.MOTOR_2] - roll_vals - pitch_vals - yaw_vals
        )
        self.current_values[MotorMap.MOTOR_3] = (
            self.current_values[MotorMap.MOTOR_3] + roll_vals + pitch_vals - yaw_vals
        )
        self.current_values[MotorMap.MOTOR_4] = (
            self.current_values[MotorMap.MOTOR_4] + roll_vals - pitch_vals + yaw_vals
        )

    def angle_expo_adj_per(self, error, expo=2, rate=15):
        input_range = (-50, 0, 50)
        output_range = (-rate, 0, rate)

        # Max change should not go above rate value (15%)
        if input_range[0] >= error:
            return -rate / 100
        elif error >= input_range[-1]:
            return rate / 100

        if error <= input_range[1]:
            norm_value = (error - input_range[1]) / (input_range[1] - input_range[0])
        else:
            norm_value = (error - input_range[1]) / (input_range[2] - input_range[1])

        if norm_value < 0:
            mapped_value = output_range[1] + (output_range[0] - output_range[1]) * (
                abs(norm_value) ** expo
            )
        else:
            mapped_value = output_range[1] + (output_range[2] - output_range[1]) * (
                norm_value**expo
            )

        return mapped_value / 100

    def acro_throttle_transformation(self):
        """
        Acrobatic Mode, changes motor throttle by up to 15% per axis. Still needs to stop the drone from spinning by using IMU Angular Velocity
        """
        # TODO

        roll_per = self.percent_map(self.crsf_channels[ROLL], EXPO)
        pitch_per = self.percent_map(self.crsf_channels[PITCH], EXPO)
        yaw_per = self.percent_map(self.crsf_channels[YAW], EXPO)

        # Note this works because all throttle values are initially the same
        roll_vals = round(self.current_values[MotorMap.MOTOR_1] * roll_per)
        pitch_vals = round(self.current_values[MotorMap.MOTOR_1] * pitch_per)
        yaw_vals = round(self.current_values[MotorMap.MOTOR_1] * yaw_per)

        self.get_logger().info(f"{roll_vals}, {pitch_vals}, {yaw_vals}")

        self.current_values[MotorMap.MOTOR_1] = (
            self.current_values[MotorMap.MOTOR_1] - roll_vals + pitch_vals + yaw_vals
        )
        self.current_values[MotorMap.MOTOR_2] = (
            self.current_values[MotorMap.MOTOR_2] - roll_vals - pitch_vals - yaw_vals
        )
        self.current_values[MotorMap.MOTOR_3] = (
            self.current_values[MotorMap.MOTOR_3] + roll_vals + pitch_vals - yaw_vals
        )
        self.current_values[MotorMap.MOTOR_4] = (
            self.current_values[MotorMap.MOTOR_4] + roll_vals - pitch_vals + yaw_vals
        )

    def quaternion_to_euler(self, quat):
        # Ripped from Online, I'm really not sure how quaternions work
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

    def percent_map(self, value, expo=2, rate=15):
        """Now with EXPO"""
        input_range = (988, 1500, 2012)
        output_range = (-rate, 0, rate)

        if not input_range[0] <= value <= input_range[-1]:
            # TODO Add failsafe on Pico to stop motors if no more pub data is recieved
            raise ValueError(
                f"Input value {value} is out of the range [{input_range[0]}, {input_range[-1]}]"
            )

        if value <= input_range[1]:
            norm_value = (value - input_range[1]) / (input_range[1] - input_range[0])
        else:
            norm_value = (value - input_range[1]) / (input_range[2] - input_range[1])

        if norm_value < 0:
            mapped_value = output_range[1] + (output_range[0] - output_range[1]) * (
                abs(norm_value) ** expo
            )
        else:
            mapped_value = output_range[1] + (output_range[2] - output_range[1]) * (
                norm_value**expo
            )

        return mapped_value / 100.0


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


if __name__ == "__main__":
    main()
