import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud
from geometry_msgs.msg import Point32
from std_msgs.msg import Float32
from example_interfaces.msg import Int64

import math

class PublishPersonLocationCount(Node):

    def __init__(self):
        super().__init__('publish_person_location_count')
        self.location_subscriber = self.create_subscription(PointCloud, 'location', self.location_callback, 10)
        self.tracking_subscriber = self.create_subscription(PointCloud, 'tracking', self.tracking_callback, 10)
        self.person_location_publisher = self.create_publisher(PointCloud, 'person_locations', 10)
        self.person_count_publisher = self.create_publisher(Int64, 'person_count', 10)

    def location_callback(self, location):
        self.get_logger().info('I heard: "%s"' % location.data)
    
    def tracking_callback(self, tracking):
        self.get_logger().info('I heard: "%s"' % tracking.data)

def main(args=None):
    rclpy.init(args=args)
    node = PublishPersonLocationCount()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
