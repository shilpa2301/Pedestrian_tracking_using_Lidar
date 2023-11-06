#Authors: Shilpa Mukhopadhyay, Amrita Mohandas, Junsuk Kim
#UIN: 433003777, 534002383, 334003508
#Instructor: Dr. Jason O'Kane
#TA: Aaron Kingery 
#Course ID: CSCE 752
#Course Title: Robotics and Spatial Intelligence
#Project 3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud
from geometry_msgs.msg import Point32
from std_msgs.msg import Int64
import numpy as np
import math
from sklearn.neighbors import KDTree

#Global parameters that should be considered in order to get promising results
THRESHOLD_CENTROIDS = 1
THRESHOLD_RADIUS_KDTREE = 0.5
THRESHOLD_OBJECT_DISTANCE = 0.8
THRESHOLD_IGNORE_DISTANCE=0.7
YOUR_DIRECTION_CHANGE_THRESHOLD=0.5
# The first node receives the laserscan data and filter outs the static data and transforms them into point cloud data.
class LaserScanNode(Node):
    def __init__(self):
        super().__init__('laser_scan_node')
        #Subscribe to LaserScan 'scan' topic and publish to an intermidate topic 'intermediate_point_cloud'.
        #Used the number 50 because did not want to lose alot of data.
        self.laser_scan_subscriber = self.create_subscription(LaserScan, 'scan', self.laser_scan_callback, 50)
        self.intermediate_publisher = self.create_publisher(PointCloud, 'intermediate_point_cloud', 50)
        #The initial frames which would be considered is a list type.
        self.initial_frames = []
         # Dictionary to track data frame counts
        self.data_frame_counts = {}
        self.last_frame=[]
    def laser_scan_callback(self, scan):
        #We would consider if the initial_frames has a smaller length than the intended length of initial frames to consider
        if len(self.initial_frames) < 4:
            # Append the scan data into the initial_frames
            self.initial_frames.append(scan)
        else:
            # If we have all the initial_frames, we now transform the scan data into point_cloud data
            intermediate_msg = self.laser_scan_to_intermediate(scan)
            #Then with the initial_frames, compare the distance with the new data and only accept which are not close to the initial_frames
            filtered_msg = self.distance_filter(intermediate_msg)
            # If there are no points at all, just end the function
            if len(filtered_msg.points) == 0:
                return
            # Use the last two frames in order to get rid of the jitter data
            self.last_frame.append(filtered_msg)
            # If it did not have at least 2 frames, return
            if len(self.last_frame)<2:
                return
            else:
                # Use the jitter filter
                jitter_msg=self.jitter_filter(filtered_msg)
                # After the filtering, pop the previous data
                self.last_frame.pop(0)
                if len(jitter_msg.points)==0:
                    return
                # Publish the filtered message and Node_2 will receive
                self.intermediate_publisher.publish(jitter_msg)  

    # A function that transforms laser data into point_cloud data
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
            # If the values are 'inf' '-inf' 'Nan' which can not be uesed, we did not use them at all.
            if ((point32.x != float('inf') and point32.x != float('-inf') and not np.isnan(point32.x)) and
                (point32.y != float('inf') and point32.y != float('-inf') and not np.isnan(point32.y)) and
                not np.isnan(point32.z)):
                point_cloud_data.append(point32) 
        point_cloud_msg = PointCloud()
        # Get the same frame_id as scan
        point_cloud_msg.header = scan.header
        point_cloud_msg.points = point_cloud_data
        return point_cloud_msg           

    # The function to filter out jitter data            
    def jitter_filter(self, point_cloud):
        filtered_points=[]
        previous_frame = self.last_frame[0]
        for i in range(len(point_cloud.points)):
            x = point_cloud.points[i].x
            y = point_cloud.points[i].y
            ignore_point = True
            for j in range(len(previous_frame.points)):
                init_x = previous_frame.points[j].x
                init_y = previous_frame.points[j].y
                distance = math.sqrt((x - init_x) ** 2 + (y - init_y) ** 2)
                # If the distance is below the ignore_threshold, we would not ignore the data
                if distance <1:
                    ignore_point = False
                
            if not ignore_point:
                filtered_points.append(point_cloud.points[i])

        distance_filtered_msg = PointCloud()
        distance_filtered_msg.header = point_cloud.header
        distance_filtered_msg.points = filtered_points
        return distance_filtered_msg

    
    # A function to filter out the static objects           
    def distance_filter(self, point_cloud):
        filtered_points = []
        for i in range(len(point_cloud.points)):
            x = point_cloud.points[i].x
            y = point_cloud.points[i].y
            ignore_point = False
            for initial_frame in self.initial_frames:
                for j in range(len(initial_frame.ranges)):
                    angle = initial_frame.angle_min + j * initial_frame.angle_increment
                    init_x = initial_frame.ranges[j] * math.cos(angle)
                    init_y = initial_frame.ranges[j] * math.sin(angle)
                    distance = math.sqrt((x - init_x) ** 2 + (y - init_y) ** 2)
                    
                    # If the distance is below the ignore_threshold, we ignore the data
                    if distance <= THRESHOLD_IGNORE_DISTANCE:
                        ignore_point = True
                        break
                if ignore_point:
                    break
            if not ignore_point:
                    filtered_points.append(point_cloud.points[i])
        distance_filtered_msg = PointCloud()
        distance_filtered_msg.header = point_cloud.header
        distance_filtered_msg.points = filtered_points
        return distance_filtered_msg
    
# The second node receives the filtered data and make cluster centroids. Also tracks objects and count the numbers.
class PointCloudNode(Node):
    def __init__(self):
        super().__init__('point_cloud_node')
        # Publish intermediate topic and publish 'person_location' and 'person_count' topic.
        self.intermediate_subscriber = self.create_subscription(PointCloud, 'intermediate_point_cloud', self.intermediate_callback, 50)
        self.centroid_publisher = self.create_publisher(PointCloud, 'person_locations', 20)
        self.human_index_publisher = self.create_publisher(Int64, 'person_count', 20)
        
        # For object tracking, use object_id_counter to compare which one is which
        self.object_id_counter = 0
        # Get a dictionary for the tracked objects
        self.tracked_objects = {}
        # The max number for number of humans
        self.human_index = 0

    # A callback function for subscribing and publishing.
    def intermediate_callback(self, msg):
        clustered_msg = self.cluster_centroid(msg)
        # Publish the centroid for each people.
        self.centroid_publisher.publish(clustered_msg)
        # Publish the number of people after the tracking function.
        self.human_index_publisher.publish(self.track_obj(clustered_msg))

    # The tracking function
    def track_obj(self,processed_msg):
        cluster_data= np.array([[point.x, point.y] for point in processed_msg.points])
        for i in range(len(cluster_data)):
            # If the cluster data is close to the previous cluster, we assume it's a previous object.
            # If the cluster data is far enough, we assume it's a new object.
            object_id = self.assign_object_id(cluster_data[i])
            print(object_id)
            # Updating the total number of tracked people.
            if object_id > self.human_index:
                self.human_index = object_id

            human_index_msg=Int64()
            human_index_msg.data=self.human_index
            # If the object_id nubmer is in the tracked objects, we update the data.
            if object_id in self.tracked_objects:
                self.tracked_objects[object_id] = cluster_data[i]
        return human_index_msg
                
    # Compare the distance and if it's below the threshold, it's an object we already know.                
    def assign_object_id(self, position):
        for obj_id, obj_position in self.tracked_objects.items():
            dist = np.sqrt((position[0] - obj_position[0]) ** 2 + (position[1] - obj_position[1]) ** 2)
            print(obj_id,obj_position,dist)
            if dist < THRESHOLD_OBJECT_DISTANCE:
                # Check the direction change as well
                prev_direction = np.arctan2(obj_position[1], obj_position[0])
                current_direction = np.arctan2(position[1], position[0])
                direction_change = np.abs(current_direction - prev_direction)
                print(obj_id,obj_position,direction_change)
                # If it is following the similar direction, it's the same object
                if direction_change < YOUR_DIRECTION_CHANGE_THRESHOLD:
                    return obj_id
        # If it's above the threshold, increase the number.
        self.object_id_counter += 1
        # Also, add the coordinates in the tracked_objects.
        self.tracked_objects[self.object_id_counter] = position
        return self.object_id_counter

    # Clustering, and then receiving the centroid
    def cluster_centroid(self, point_cloud_msg):
        data = []
        for i in range(len(point_cloud_msg.points)):
            data.append([point_cloud_msg.points[i].x, point_cloud_msg.points[i].y, point_cloud_msg.points[i].z])
        # Use KDTree in order to get centroids
        tree = KDTree(data)

        #neighbours in same cluster also given out, hence redundant cluster
        neighbors = tree.query_radius(data, r=THRESHOLD_RADIUS_KDTREE)
        centroids = []
        x = 0.0
        y = 0.0
        z = 0.0

        #first data, simply append
        if len(centroids) == 0:
            for i in range(len(neighbors[0])):
                x += data[neighbors[0][i]][0]
                y += data[neighbors[0][i]][1]
                z += data[neighbors[0][i]][2]
            value_list = [x / len(neighbors[0]), y / len(neighbors[0]), z / len(neighbors[0])]
            centroids.append(value_list)


        #decide how to add remaining neighbors to remove redundant clusters
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

            #redundant clusters. action -> take the mean with the current centroid
            if euc_dist <= THRESHOLD_CENTROIDS:
                mean_x = (centre[0] + centroids[least_dist_id][0]) / 2.0
                mean_y = (centre[1] + centroids[least_dist_id][1]) / 2.0
                mean_z = (centre[2] + centroids[least_dist_id][2]) / 2.0
                centroids[least_dist_id] = [mean_x, mean_y, mean_z]
            else:
            #non redundant cluster identified, hence simply append
                centroids.append(centre)

        #convert to point and append to point cloud for future processing
        data_cloud = []
        for cen in centroids:
            centroids_Point = Point32()
            centroids_Point.x = cen[0]
            centroids_Point.y = cen[1]
            centroids_Point.z = cen[2]
            data_cloud.append(centroids_Point)
        centroids.clear()
        point_cloud_msg_ = PointCloud()
        point_cloud_msg_.header = point_cloud_msg.header
        point_cloud_msg_.points = data_cloud
        return point_cloud_msg_
# Since we only have one python file, we used two main functions.
# The first one is for execution 1 and the other is for execution 2.
def main(args=None):
    rclpy.init(args=args)
    print('node_1 start')
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
