import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud
from geometry_msgs.msg import Point32
from std_msgs.msg import Int64
import numpy as np
import math
from sklearn.neighbors import KDTree

THRESHOLD_CENTROIDS = 1
THRESHOLD_RADIUS_KDTREE = 0.5
THRESHOLD_OBJECT_DISTANCE = 1

class LaserScanNode(Node):

    def __init__(self):
        super().__init__('laser_scan_node')
        self.frame_id = 0
        self.laser_scan_subscriber = self.create_subscription(LaserScan, 'scan', self.laser_scan_callback, 10)
        self.intermediate_publisher = self.create_publisher(PointCloud, 'intermediate_point_cloud', 10)
        self.centroid_publisher = self.create_publisher(PointCloud, 'centroid', 10)
        self.human_index_publisher = self.create_publisher(Int64, 'people_number', 10)
        
        self.object_id_counter = 0
        self.tracked_objects = {}
        self.human_positions = {}
        self.human_index = 0
        self.initial_frames = []
        self.ignore_distance = 0.5
        

    def laser_scan_callback(self, scan):
        if len(self.initial_frames) < 5:
            self.initial_frames.append(scan)
        else:
            intermediate_msg = self.laser_scan_to_intermediate(scan)
            roi_filtered_msg = self.distance_filter(intermediate_msg)
            self.intermediate_publisher.publish(intermediate_msg)
            if len(roi_filtered_msg.points) == 0:
                return
            processed_msg = self.process_data(roi_filtered_msg)
            self.centroid_publisher.publish(processed_msg)

            cluster_data = np.array([[point.x, point.y] for point in processed_msg.points])
        
            for i in range(len(cluster_data)):
                object_id = self.assign_object_id(cluster_data[i])
                if object_id > self.human_index:
                    self.human_index = object_id
                self.get_logger().info(f"Object ID: {object_id}")
                
    def distance_filter(self, scan):
        filtered_points = []

        for i in range(len(scan.points)):
            x = scan.points[i].x
            y = scan.points[i].y
            ignore_point = False

            for initial_frame in self.initial_frames:
                for j in range(len(initial_frame.ranges)):
                    angle = initial_frame.angle_min + j * initial_frame.angle_increment
                    init_x = initial_frame.ranges[j] * math.cos(angle)
                    init_y = initial_frame.ranges[j] * math.sin(angle)

                    distance = math.sqrt((x - init_x) ** 2 + (y - init_y) ** 2)
                    if distance <= self.ignore_distance:
                        ignore_point = True
                        break

                if ignore_point:
                    break

            if not ignore_point:
                filtered_points.append(scan.points[i])

        distance_filtered_msg = PointCloud()
        distance_filtered_msg.header = scan.header
        distance_filtered_msg.points = filtered_points

        return distance_filtered_msg

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

            if ((point32.x != float('inf') and point32.x != float('-inf') and not np.isnan(point32.x)) and
                (point32.y != float('inf') and point32.y != float('-inf') and not np.isnan(point32.y)) and
                not np.isnan(point32.z)):
                point_cloud_data.append(point32)

        self.frame_id = scan.header
        point_cloud_msg = PointCloud()
        point_cloud_msg.header = self.frame_id
        point_cloud_msg.points = point_cloud_data

        return point_cloud_msg

    def assign_object_id(self, position):
        for obj_id, obj_position in self.tracked_objects.items():
            dist = np.sqrt((position[0] - obj_position[0]) ** 2 + (position[1] - obj_position[1]) ** 2)
            if dist < THRESHOLD_OBJECT_DISTANCE:
                return obj_id

        self.object_id_counter += 1
        self.tracked_objects[self.object_id_counter] = position
        return self.object_id_counter

    def process_data(self, point_cloud_msg):
        data = []
        for i in range(len(point_cloud_msg.points)):
            data.append([point_cloud_msg.points[i].x, point_cloud_msg.points[i].y, point_cloud_msg.points[i].z])

        tree = KDTree(data)
        neighbors = tree.query_radius(data, r=THRESHOLD_RADIUS_KDTREE)
        centroids = []
        x = 0.0
        y = 0.0
        z = 0.0

        if len(centroids) == 0:
            for i in range(len(neighbors[0])):
                x += data[neighbors[0][i]][0]
                y += data[neighbors[0][i]][1]
                z += data[neighbors[0][i]][2]
            value_list = [x / len(neighbors[0]), y / len(neighbors[0]), z / len(neighbors[0])]
            centroids.append(value_list)

        for n in range(1, len(neighbors)):
            x = 0.0
            y = 0.0
            z = 0.0
            for i in range(len(neighbors[n])):
                x += data[neighbors[n][i]][0]
                y += data[neighbors[n][i]][1]
                z += data[neighbors[n][i]][2]
            centre = [x / len(neighbors[n]), y / len(neighbors[n]), z / len(neighbors[n])]
            least_dist = float('inf')
            least_dist_id = -1
            for c in range(len(centroids)):
                euc_dist = np.sqrt((centre[0] - centroids[c][0]) ** 2 + (centre[1] - centroids[c][1]) ** 2 +
                                  (centre[2] - centroids[c][2]) ** 2)
                if euc_dist < least_dist:
                    least_dist = euc_dist
                    least_dist_id = c
            if euc_dist <= THRESHOLD_CENTROIDS:
                mean_x = (centre[0] + centroids[least_dist_id][0]) / 2.0
                mean_y = (centre[1] + centroids[least_dist_id][1]) / 2.0
                mean_z = (centre[2] + centroids[least_dist_id][2]) / 2.0
                centroids[least_dist_id] = [mean_x, mean_y, mean_z]
            else:
                centroids.append(centre)
        data_cloud = []
        for cen in centroids:
            centroids_Point = Point32()
            centroids_Point.x = cen[0]
            centroids_Point.y = cen[1]
            centroids_Point.z = cen[2]
            data_cloud.append(centroids_Point)
        centroids.clear()
        point_cloud_msg_ = PointCloud()
        point_cloud_msg_.header = self.frame_id
        point_cloud_msg_.points = data_cloud
        return point_cloud_msg_

def main(args=None):
    rclpy.init(args=args)
    node = LaserScanNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
