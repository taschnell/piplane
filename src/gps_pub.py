import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
import gps

class GPSDClient(Node):
    def __init__(self):
        super().__init__('gpsd_client')
        self.publisher_ = self.create_publisher(NavSatFix, 'gps/fix', 1)
        self.session = gps.gps(mode=gps.WATCH_ENABLE)
        self.timer = self.create_timer(0.2, self.timer_callback)
        self.get_logger().info('GPSD client node started.')

    def timer_callback(self):
        if self.session.read() == 0 and (self.session.valid):
            msg = NavSatFix()
            if gps.TIME_SET & self.session.valid:
                msg.header.stamp = self.get_clock().now().to_msg()
            else:
                msg.header.stamp = rclpy.time.Time().to_msg()
            
            msg.latitude = self.session.fix.latitude if gps.isfinite(self.session.fix.latitude) else float('nan')
            msg.longitude = self.session.fix.longitude if gps.isfinite(self.session.fix.longitude) else float('nan')
            msg.altitude = self.session.fix.altHAE if gps.isfinite(self.session.fix.altHAE) else float('nan')

            msg.position_covariance = [0.0] * 9
            msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN

            self.publisher_.publish(msg)
            self.get_logger().info(f'Published GPS data: Lat {msg.latitude}, Lon {msg.longitude}, Alt {msg.altitude}')
        else:
            self.get_logger().warn('No valid GPS data received.')

def main(args=None):
    rclpy.init(args=args)
    node = GPSDClient()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt, shutting down.')

    node.session.close()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
