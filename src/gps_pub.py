import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix  # ROS 2 message type for GPS data
import gps


class GPSNode(Node):
    def __init__(self):
        super().__init__("gps")
        self.publisher_ = self.create_publisher(NavSatFix, "gps_data", 1)
        self.session = gps.gps(mode=gps.WATCH_ENABLE)

    def run_gps(self):
        while not self.session.waiting(0.2):
            if not (gps.MODE_SET & self.session.valid):
                continue

            msg = NavSatFix()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "gps_frame"
            msg.status.service = NavSatFix.SERVICE_GPS

            msg.latitude = self.session.fix.latitude
            msg.longitude = self.session.fix.longitude
            msg.altitude = self.session.fix.altMSL
            msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
            msg.position_covariance = [0.0] * 9

            self.publisher_.publish(msg)
            self.get_logger().info("Publishing GPS data")


def main(args=None):
    rclpy.init(args=args)
    node = GPSNode()
    try:
        node.run_gps()
    except KeyboardInterrupt:
        pass
    finally:
        node.session.close()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
