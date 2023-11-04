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
from std_msgs.msg import Int64
from std_msgs.msg import Int32  # 새로운 메시지 추가
THRESHOLD_CENTROIDS = 1
THRESHOLD_RADIUS_KDTREE = 0.5
HUMAN_DIRECTION_THRESHOLD = 30.0  # 사람으로 인식하는 방향의 임계값
THRESHOLD_OBJECT_DISTANCE=1
DISTANCE_THRESHOLD=0.3
class LaserScanNode(Node):

    def __init__(self):
        super().__init__('laser_scan_node')
        self.frame = 0
        self.frame_id=0
        self.laser_scan_subscriber = self.create_subscription(LaserScan, 'scan', self.laser_scan_callback, 10)
        self.intermediate_publisher = self.create_publisher(PointCloud, 'intermediate_point_cloud', 10)
        self.centroid_publisher = self.create_publisher(PointCloud, 'centroid', 10)
        self.human_direction_publisher = self.create_publisher(PointCloud, 'human_direction', 10)
        self.human_index_publisher=self.create_publisher(Int64,'people_number',10)
        
        # Kalman filter initialization
        self.kf = KalmanFilter(dim_x=2, dim_z=1)
        self.kf.F = np.array([[1., 1.], [0., 1.]])  # State transition matrix
        self.kf.H = np.array([[1., 0.]])  # Measurement function
        self.kf.P *= 100.0  # Initial state covariance matrix
        self.kf.R = 10.0  # Measurement noise
        self.kf.Q = Q_discrete_white_noise(dim=2, dt=0.1, var=0.1)  # Process noise
        self.object_id_counter = 0
        self.tracked_objects = {}
        self.human_positions = {}
        self.frames_since_detection = 0
        self.human_index = 0
        self.human_direction = 0.0
        self.initial_frames = []  # 처음 5개 프레임 저장
        self.ignore_distance = 0.5  # 0.1 이하의 거리에 있는 점은 무시

        
        
    def laser_scan_callback(self, scan):
        if len(self.initial_frames) < 5:
            # 처음 시작 5개의 프레임은 초기 프레임으로 저장
            self.initial_frames.append(scan)
        else:
            # 처음 시작 5개의 프레임 후에 ROI 필터링 적용
            intermediate_msg = self.laser_scan_to_intermediate(scan)
            # 초기 프레임으로부터 ROI 내의 데이터만 추출
            
            roi_filtered_msg = self.distance_filter(intermediate_msg)
            self.intermediate_publisher.publish(intermediate_msg)
            if len(roi_filtered_msg.points)==0:
                return
            processed_msg = self.process_data(roi_filtered_msg)
            self.centroid_publisher.publish(processed_msg)

        # Calculate cluster direction
            cluster_data = np.array([[point.x, point.y] for point in processed_msg.points])
            data_cloud = []
    
        # 클러스터 이동을 추적할 변수
            for i in range(len(cluster_data)):
                cluster_direction_radians = np.arctan2(cluster_data[i][1], cluster_data[i][0])
                cluster_direction_degrees = np.degrees(cluster_direction_radians)
                self.get_logger().info(f"Cluster Direction (Degrees): {cluster_direction_degrees}")
    
            # Predict object direction using Kalman filter
                self.kf.predict()
                self.kf.update(cluster_direction_degrees)
                object_direction_degrees = self.kf.x[0]
                self.get_logger().info(f"Object Direction (Degrees): {object_direction_degrees}")
 # .    ..
    
                # ID 할당 및 업데이트
                object_id = self.assign_object_id(cluster_data[i])
                if object_id>self.human_index:
                    self.human_index=object_id
                self.get_logger().info(f"Object ID: {object_id}")
                
                if object_id in self.tracked_objects:
                    # 현재 프레임에서 감지된 물체의 위치를 저장
                    self.tracked_objects[object_id] = cluster_data[i]
                human_index_msg = Int64()
                human_index_msg.data = self.human_index
                self.human_index_publisher.publish(human_index_msg)
                # # 물체가 사람으로 판단될 경우 사람의 이동을 추적
                #     if abs(object_direction_degrees - self.human_direction) < HUMAN_DIRECTION_THRESHOLD:
                #         if object_id in self.human_positions:
                #             prev_position = self.human_positions[object_id]
                #             distance = np.sqrt((cluster_data[i][0] - prev_position[0]) ** 2 + (cluster_data[i][1] - prev_position[1]) ** 2)
                #             if distance > DISTANCE_THRESHOLD:  # 이동 거리가 10 이상인 경우에만 사람으로 판단
                #                 self.get_logger().info(f"Moving human detected (ID: {object_id})")
                #                 self.human_index += 1  # 사람 인덱스 증가
    
                #                 # 필요한 경우 사람 인덱스를 publish
                #                 human_index_msg = Int64()
                #                 human_index_msg.data = self.human_index
                #                 self.human_index_publisher.publish(human_index_msg)
                #     # 현재 위치를 이전 위치로 업데이트
                #         self.human_positions[object_id] = cluster_data[i]
                # else:
                # # 사람으로 판단된 물체가 아닐 경우, 물체의 ID를 갱신하지 않고 추적하지 않음
                # # 이동이 없는 물체는 여기에 해당
                #     pass
    
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
            
    def distance_filter(self, scan):
        # 처음 5개의 프레임에서 얻은 점과의 거리가 일정 이상인 데이터만 추출하여 반환
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

                    # 두 점 간의 거리 계산
                    distance = math.sqrt((x - init_x) ** 2 + (y - init_y) ** 2)
                    if distance <= self.ignore_distance:
                        ignore_point = True
                        break

                if ignore_point:
                    break

            if not ignore_point:
                filtered_points.append(scan.points[i])
        # 거리 필터링된 데이터로 새로운 PointCloud 메시지 생성
        distance_filtered_msg = PointCloud()
        distance_filtered_msg.header = scan.header
        distance_filtered_msg.points = filtered_points

        return distance_filtered_msg

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
    def assign_object_id(self, position):
        # 새로운 물체의 중심 위치를 받아서 ID를 할당하거나 업데이트합니다.
        for obj_id, obj_position in self.tracked_objects.items():
            dist = np.sqrt((position[0] - obj_position[0]) ** 2 + (position[1] - obj_position[1]) ** 2)
            if dist < THRESHOLD_OBJECT_DISTANCE:
                # 이미 추적 중인 물체와 거리가 가까우면 해당 물체의 ID 반환
                return obj_id

        # 새로운 물체인 경우, 새로운 ID를 생성하고 저장
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
