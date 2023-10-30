import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud
from geometry_msgs.msg import Point32
from std_msgs.msg import Float32
import numpy as np
from filterpy.kalman import KalmanFilter
import math
from filterpy.common import Q_discrete_white_noise
from sklearn.neighbors import KDTree
from std_msgs.msg import Int64
THRESHOLD_CENTROIDS = 1
THRESHOLD_RADIUS_KDTREE = 1
HUMAN_DIRECTION_THRESHOLD = 30.0  # 사람으로 인식하는 방향의 임계값

class LaserScanNode(Node):

    def __init__(self):
        super().__init__('laser_scan_node')
        self.frame = 0
        self.frame_id=0
        self.laser_scan_subscriber = self.create_subscription(LaserScan, 'scan', self.laser_scan_callback, 10)
        self.intermediate_publisher = self.create_publisher(PointCloud, 'intermediate_point_cloud', 10)
        self.centroid_publisher = self.create_publisher(PointCloud, 'centroid', 10)
        self.human_direction_publisher = self.create_publisher(PointCloud, 'human_direction', 10)

        # Kalman filter initialization
        self.kf = KalmanFilter(dim_x=2, dim_z=1)
        self.kf.F = np.array([[1., 1.], [0., 1.]])  # State transition matrix
        self.kf.H = np.array([[1., 0.]])  # Measurement function
        self.kf.P *= 100.0  # Initial state covariance matrix
        self.kf.R = 10.0  # Measurement noise
        self.kf.Q = Q_discrete_white_noise(dim=2, dt=0.1, var=0.1)  # Process noise



        self.human_direction = 0.0

    def laser_scan_callback(self, scan):
        intermediate_msg = self.laser_scan_to_intermediate(scan)
        processed_msg = self.process_data(intermediate_msg)
        self.centroid_publisher.publish(processed_msg)
        self.intermediate_publisher.publish(intermediate_msg)
        # Calculate cluster direction
        cluster_data = np.array([[point.x, point.y] for point in processed_msg.points])
        print(len(cluster_data))
        data_cloud=[]
        for i in range(len(cluster_data)):
            cluster_direction_radians = np.arctan2(cluster_data[i][1], cluster_data[i][0])
            cluster_direction_degrees = np.degrees(cluster_direction_radians)
            self.get_logger().info(f"Cluster Direction (Degrees): {cluster_direction_degrees}")

        # Predict object direction using Kalman filter
            self.kf.predict()
            self.kf.update(cluster_direction_degrees)
            object_direction_degrees = self.kf.x[0]
            self.get_logger().info(f"Object Direction (Degrees): {object_direction_degrees}")

        # # Publish object direction
        # direction_msg = Int64()
        # direction_msg.data = int(object_direction_degrees)

            print(abs(object_direction_degrees - self.human_direction))
        # Check if the cluster direction is within the threshold to be considered a human
        
            if abs(object_direction_degrees - self.human_direction) < HUMAN_DIRECTION_THRESHOLD:
                self.get_logger().info(f"Human detected._{cluster_data[i]}")
                point32 = Point32()
                point32.x = cluster_data[i][0]
                point32.y = cluster_data[i][1]
                point32.z = 0.0
                data_cloud.append(point32)
                point_cloud_msg_ = PointCloud()
                point_cloud_msg_.header = self.frame_id
                point_cloud_msg_.points = data_cloud
                self.human_direction_publisher.publish(point_cloud_msg_)
            # Add your code to perform actions when a human is detected

    def laser_scan_to_intermediate(self, scan):
        point_cloud_data = []
        point_original = []
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
            point_original.append(point32)
        self.frame_id=scan.header
        point_cloud_msg = PointCloud()
        point_cloud_msg.header = self.frame_id
        
        point_cloud_msg.points = point_cloud_data
        return point_cloud_msg

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
