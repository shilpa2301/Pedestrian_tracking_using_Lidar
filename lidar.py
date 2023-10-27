import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud
from geometry_msgs.msg import Point32
from std_msgs.msg import Int64
import numpy as np

import math
from sklearn.neighbors import KDTree
THRESHOLD_CENTROIDS =  1.0
class LaserScanNode(Node):

    def __init__(self):
        super().__init__('laser_scan_node')
        self.laser_scan_subscriber = self.create_subscription(LaserScan, 'scan', self.laser_scan_callback, 10)
        self.intermediate_publisher = self.create_publisher(PointCloud, 'intermediate_point_cloud', 10)
        self.centroid_publisher = self.create_publisher(PointCloud, 'centroid', 10)

    def laser_scan_callback(self, scan):
        intermediate_msg = self.laser_scan_to_intermediate(scan)
        processed_msg=self.process_data(intermediate_msg)
        self.centroid_publisher.publish(processed_msg)
        self.intermediate_publisher.publish(intermediate_msg)

    def laser_scan_to_intermediate(self, scan):
        point_cloud_data = []
        point_original=[]
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
            point_original.append(point32)
        point_cloud_msg = PointCloud()
        point_cloud_msg.header = scan.header
        point_cloud_msg.points = point_cloud_data

        return point_cloud_msg
    def process_data(self, point_cloud_msg):
        data_dummy=[]
        data_cloud=[]
        for i in range(len(point_cloud_msg.points)):
            data_dummy.append([point_cloud_msg.points[i].x, point_cloud_msg.points[i].y, point_cloud_msg.points[i].z])
        tree = KDTree(data_dummy)
        neighbors = tree.query_radius(data_dummy, r = 1.0)
        centroids=[]
        x=0.0
        y=0.0
        z=0.0
        centroids_Point=Point32()
        if len(centroids)==0:
            for i in range(len(neighbors[0])):
                x+=data_dummy[neighbors[0][i]][0]
                y+=data_dummy[neighbors[0][i]][1]
                z+=data_dummy[neighbors[0][i]][2]
            value_list=[x/len(neighbors[0]), y/len(neighbors[0]), z/len(neighbors[0])]
            centroids.append(value_list)
            centroids_Point.x=value_list[0]
            centroids_Point.y=value_list[1]
            centroids_Point.z=value_list[2]
            
            data_cloud.append(centroids_Point)
                
        for n in range(1,len(neighbors)):
            x=0.0
            y=0.0
            z=0.0
            for i in range(len(neighbors[n])):
                x+=data_dummy[neighbors[n][i]][0]
                y+=data_dummy[neighbors[n][i]][1]
                z+=data_dummy[neighbors[n][i]][2]
            centre = [x/len(neighbors[n]), y/len(neighbors[n]), z/len(neighbors[n])]
            #centroids.append([x/len(neighbors[n]), y/len(neighbors[n]), z/len(neighbors[n])])
            least_dist = float('inf')
            least_dist_id = -1
            for c in range(len(centroids)):
                euc_dist = np.sqrt((centre[0]-centroids[c][0])**2 + (centre[1]-centroids[c][1])**2 + (centre[2]-centroids[c][2])**2)
                if euc_dist < least_dist:
                    least_dist = euc_dist
                    least_dist_id=c
            #print ("least_dist=", least_dist)
            if euc_dist <= THRESHOLD_CENTROIDS:
                mean_x = (centre[0] + centroids[least_dist_id][0]) /2.0
                mean_y = (centre[1] + centroids[least_dist_id][1]) /2.0
                mean_z = (centre[2] + centroids[least_dist_id][2]) /2.0
                centroids[least_dist_id] = [mean_x, mean_y, mean_z]
                centroids_Point.x=mean_x
                centroids_Point.y=mean_y
                centroids_Point.z=mean_z
            else:    
                centroids.append(centre)
                centroids_Point.x=centre[0]
                centroids_Point.y=centre[1]
                centroids_Point.z=centre[2]
                data_cloud.append(centroids_Point)
        data_cloud.append(centroids_Point)
        centroids.clear()
        #print("num_neighbours= ", num_neighbors)
        point_cloud_msg_ = PointCloud()
        point_cloud_msg_.header = point_cloud_msg.header
        point_cloud_msg_.points = data_cloud
        return point_cloud_msg_
        
        
        
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
