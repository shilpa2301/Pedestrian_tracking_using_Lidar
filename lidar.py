import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud
from geometry_msgs.msg import Point32
from std_msgs.msg import Int64
import numpy as np

import math
from sklearn.neighbors import KDTree
class LaserScanNode(Node):

    def __init__(self):
        super().__init__('laser_scan_node')
        self.laser_scan_subscriber = self.create_subscription(LaserScan, 'scan', self.laser_scan_callback, 10)
        self.intermediate_publisher = self.create_publisher(PointCloud, 'intermediate_point_cloud', 10)

    def laser_scan_callback(self, scan):
        intermediate_msg = self.laser_scan_to_intermediate(scan)
        #processed_msg=self.process_data(intermediate_msg)
        self.intermediate_publisher.publish(intermediate_msg)

    def laser_scan_to_intermediate(self, scan):
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
            if ((point32.x!=float('inf') and point32.x!=float('-inf') and np.isnan(point32.x)==False) \
            and (point32.y!=float('inf') and point32.y!=float('-inf') and np.isnan(point32.y)==False) \
            and point32.z!=float('inf') and point32.z!=float('-inf') and np.isnan(point32.z)==False) :
                point_cloud_data.append(point32)

        point_cloud_msg = PointCloud()
        point_cloud_msg.header = scan.header
        point_cloud_msg.points = point_cloud_data

        return point_cloud_msg
    def process_data(self, point_cloud_msg):
        data_cloud=[]
        data_dummy=[]
        #print("before removing=",len(point_cloud_msg.points))
        for i in range(len(point_cloud_msg.points)):
            #print("i-th data= ",point_cloud_msg.points[i].x,',',point_cloud_msg.points[i].y)
            if ((point_cloud_msg.points[i].x!=float('inf') and point_cloud_msg.points[i].x!=float('-inf') and np.isnan(point_cloud_msg.points[i].x)==False) \
            and (point_cloud_msg.points[i].y!=float('inf') and point_cloud_msg.points[i].y!=float('-inf') and np.isnan(point_cloud_msg.points[i].y)==False) \
            and point_cloud_msg.points[i].z!=float('inf') and point_cloud_msg.points[i].z!=float('-inf') and np.isnan(point_cloud_msg.points[i].z)==False) :
                data=Point32()
                data.x=point_cloud_msg.points[i].x
                data.y=point_cloud_msg.points[i].y
                data.z=point_cloud_msg.points[i].z
                # print([point_cloud_msg.points[i].x, point_cloud_msg.points[i].y, point_cloud_msg.points[i].z])
                data_cloud.append(data)
                # print([point_cloud_msg.points[i].x, point_cloud_msg.points[i].y, point_cloud_msg.points[i].z])
                data_dummy.append([point_cloud_msg.points[i].x, point_cloud_msg.points[i].y, point_cloud_msg.points[i].z])
        #print("data=",data)
        #print("length of data=", len(data))
        tree = KDTree(data_dummy)
        num_neighbors = tree.query_radius(data_dummy, r = 1.0)
        #print("num_neighbours= ", num_neighbors)
        point_cloud_msg = PointCloud()
        point_cloud_msg.header = point_cloud_msg.header
        point_cloud_msg.points = data_cloud
        return point_cloud_msg
        
        
        
class PointCloudNode(Node):
    def __init__(self):
        super().__init__('point_cloud_node')
        self.intermediate_subscriber = self.create_subscription(PointCloud, 'intermediate_point_cloud', self.intermediate_callback, 10)
        self.point_cloud_publisher = self.create_publisher(PointCloud, 'point_cloud', 10)
        self.people_publisher = self.create_publisher(Int64, 'people_number', 10)  # Create a publisher for Int64

    def intermediate_callback(self, msg):
        point_cloud_msg = self.intermediate_to_point_cloud(msg)
        self.point_cloud_publisher.publish(point_cloud_msg)
        int64_msg=self.intermediate_to_number()
        self.people_publisher.publish(int64_msg)

    def intermediate_to_number(self):
        int64_msg = Int64()
        int64_msg.data = 42 
        return int64_msg 
    def intermediate_to_point_cloud(self, msg):
        point_cloud_data=[]
        for i in range(len(msg.points)):
            
            point32 = Point32()
            point32.x = msg.points[i].x
            point32.y = msg.points[i].y
            point32.z = msg.points[i].z
            point_cloud_data.append(point32)
            
        point_cloud_msg=PointCloud()
        point_cloud_msg.points=point_cloud_data
        point_cloud_msg.header=msg.header
        return point_cloud_msg


        
def main(args=None):
    rclpy.init(args=args)
    print('node_1  start')
    node = LaserScanNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

def main_2(args=None):
    rclpy.init(args=args)
    print('node_2 start')
    node_2=PointCloudNode()
    rclpy.spin(node_2)
    node_2.destroy_node()
    rclpy.shutdown()
# if __name__ == '__main__':
#     main()
