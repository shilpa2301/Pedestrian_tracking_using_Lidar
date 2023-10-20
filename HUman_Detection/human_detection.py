import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud
from geometry_msgs.msg import Point32
import math
from std_msgs.msg import Float32
from example_interfaces.msg import Int64


class Human_Detection( Node):
    def __init__(self,node_name):
        super().__init__(node_name)
        self.laser_scan_subscriber = self.create_subscription(LaserScan, '/scan', self.laser_scan_callback, 10)
        self.point_cloud_publisher = self.create_publisher(PointCloud, '/point_cloud', 10)

        #self.laser_scan_subscriber = self.create_subscription(LaserScan, '/scan', self.laser_scan_callback, 10)
        self.location_publisher = self.create_publisher(PointCloud, '/location', 10)
        self.tracking_publisher = self.create_publisher(PointCloud, '/tracking', 10)

        self.location_subscriber = self.create_subscription(PointCloud, '/location', self.location_callback, 10)
        self.tracking_subscriber = self.create_subscription(PointCloud, '/tracking', self.tracking_callback, 10)
        self.person_location_publisher = self.create_publisher(PointCloud, '/person_locations', 10)
        self.person_count_publisher = self.create_publisher(Int64, '/person_count', 10)

    def laser_scan_callback(self, scan):
        point_cloud_msg = self.laser_scan_to_point_cloud(scan)
        self.point_cloud_publisher.publish(point_cloud_msg)
        print("published_data=", point_cloud_msg)
    
    def location_callback(self, location):
        self.get_logger().info('I heard: "%s"' % location.data)
    
    def tracking_callback(self, tracking):
        self.get_logger().info('I heard: "%s"' % tracking.data)


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

    #def get_data(self):
        


def main(args=None):
    rclpy.init(args=args)
    node1 = Human_Detection('Node1')
    rclpy.spin(node1)
    node1.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()