import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud
from geometry_msgs.msg import Point32
from std_msgs.msg import Float32

import math

class LaserScanSubscriber(Node):

    def __init__(self):
        super().__init__('laser_scan_sub_node')
        self.laser_scan_subscriber = self.create_subscription(LaserScan, '/scan', self.laser_scan_callback, 10)
        # publisher for location
        self.location_publisher = self.create_publisher(PointCloud, '/location', 10)
        # publisher for tracking
        self.tracking_publisher = self.create_publisher(PointCloud, '/tracking', 10)

    def laser_scan_callback(self, scan):
        self.get_logger().info('I heard: "%s"' % scan.data)

    def get_location(self, scan):
        location_msg = 0
        return location_msg
    
    def get_tracking(self, scan):
        tracking_msg = 0
        return tracking_msg

def main(args=None):
    rclpy.init(args=args)
    node = LaserScanSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()