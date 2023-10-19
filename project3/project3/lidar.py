import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud
from geometry_msgs.msg import Point32

import math

class LaserScanToPointCloudNode(Node):

    def __init__(self):
        super().__init__('laser_scan_to_point_cloud_node')
        self.laser_scan_subscriber = self.create_subscription(LaserScan, 'scan', self.laser_scan_callback, 10)
        self.point_cloud_publisher = self.create_publisher(PointCloud, 'point_cloud', 10)

    def laser_scan_callback(self, scan):
        point_cloud_msg = self.laser_scan_to_point_cloud(scan)
        self.point_cloud_publisher.publish(point_cloud_msg)

    def laser_scan_to_point_cloud(self, scan):
        point_cloud_data = []

        for i in range(len(scan.ranges)):
            angle = scan.angle_min + i * scan.angle_increment
            x = scan.ranges[i] * math.cos(angle)
            y = scan.ranges[i] * math.sin(angle)
            z = 0.0

            point32 = Point32()
            point32.x = x
            point32.y = y
            point32.z = z
            point_cloud_data.append(point32)

        point_cloud_msg = PointCloud()
        point_cloud_msg.header = scan.header
        point_cloud_msg.points = point_cloud_data

        return point_cloud_msg

def main(args=None):
    rclpy.init(args=args)
    node = LaserScanToPointCloudNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
