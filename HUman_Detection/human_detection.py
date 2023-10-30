import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud
from geometry_msgs.msg import Point32
from std_msgs.msg import Int64
import numpy as np

import math
from sklearn.neighbors import KDTree

import numpy.linalg as LA

THRESHOLD_SENSOR_MATCH = 0.2
THRESHOLD_CENTROIDS =  0.3
THRESHOLD_RADIUS_KDTREE = 0.5
THRESHOLD_IOU = 0.95
THRESHOLD_INTERSECTION = 0.90
#FRAME =0

class LaserScanNode(Node):

    def __init__(self):
        super().__init__('laser_scan_node')
        #self.frame = 0
        self.first_frame_data=[]
        self.common_data =[]

        self.laser_scan_subscriber = self.create_subscription(LaserScan, 'scan', self.laser_scan_callback, 10)
        self.intermediate_publisher = self.create_publisher(PointCloud, 'intermediate_point_cloud', 10)
        self.centroid_publisher = self.create_publisher(PointCloud, 'centroid', 10)

        #create subscriber to /frame id topic
        self.frame_subscriber = self.create_subscription(Int64, 'frame_id', self.frame_callback, 10)
        self.frame_publisher = self.create_publisher(Int64, 'frame_id', 10)

    def frame_callback(self, frame_id):
        self.frame_id = frame_id
        return frame_id

    def bb_Intersection(boxA, boxB):
        xA = max(boxA[0], boxB[0])
        yA = max(boxA[1], boxB[1])
        xB = min(boxA[2], boxB[2])
        yB = min(boxA[3], boxB[3])
        # compute the area of intersection rectangle
        interArea = max(0, xB - xA + 1) * max(0, yB - yA + 1)

        return interArea

    def bb_IOU(boxA, boxB):
        # determine the (x, y)-coordinates of the intersection rectangle
        xA = max(boxA[0], boxB[0])
        yA = max(boxA[1], boxB[1])
        xB = min(boxA[2], boxB[2])
        yB = min(boxA[3], boxB[3])
        # compute the area of intersection rectangle
        interArea = max(0, xB - xA + 1) * max(0, yB - yA + 1)
        # compute the area of both the prediction and ground-truth
        # rectangles
        boxAArea = (boxA[2] - boxA[0] + 1) * (boxA[3] - boxA[1] + 1)
        boxBArea = (boxB[2] - boxB[0] + 1) * (boxB[3] - boxB[1] + 1)
        # compute the intersection over union by taking the intersection
        # area and dividing it by the sum of prediction + ground-truth
        # areas - the interesection area
        iou = interArea / float(boxAArea + boxBArea - interArea)
        # return the intersection over union value
        return iou

    def laser_scan_callback(self, scan):
        intermediate_msg = self.laser_scan_to_intermediate(scan)
        processed_msg=self.process_data(intermediate_msg)
        self.centroid_publisher.publish(processed_msg[0])
        self.intermediate_publisher.publish(processed_msg[1])


    def find_static_common_points(self, ref_frame_data, curr_frame_data):
        common_data = []
        bb_list =[]
        centr_list =[]

        for i in range(len(ref_frame_data[0])):
            for j in range(len(curr_frame_data[0])):
                iou = self.bb_IOU(ref_frame_data[1][i], curr_frame_data[1][j])

                if iou > THRESHOLD_IOU:
                    x_low =  min(ref_frame_data[1][i][0],curr_frame_data[1][j][0])
                    x_high = max(ref_frame_data[1][i][2],curr_frame_data[1][j][2])
                    y_low =  min(ref_frame_data[1][i][1],curr_frame_data[1][j][1])
                    y_high = max(ref_frame_data[1][i][3],curr_frame_data[1][j][3])

                    centroid_mean = ((ref_frame_data[0][i]+curr_frame_data[0][j])/2.0)

                    bb_list.append([x_low, y_low, x_high, y_high])
                    centr_list.append(centroid_mean)
                    del ref_frame_data[1][i]
                    del curr_frame_data[1][j]
                    del ref_frame_data[0][i]
                    del curr_frame_data[0][j]
                

        common_data.append([centr_list, bb_list])        
        return common_data

    def laser_scan_to_intermediate(self, scan):
        #global FRAME
        #FRAME += 1
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
    
    def dynamic_point_data(self, common_data, data):

        dynamic_data = []
        
        bb_list =[]
        centr_list =[]

        for i in range(len(common_data[0])):
            for j in range(len(data[0])):
                intersection = self.bb_Intersection(common_data[1][i], data[1][j])

                if intersection > THRESHOLD_INTERSECTION:
                    del data[0][j]
                    del data[1][j]
                                        
        dynamic_data = data       
        
        return dynamic_data


    def process_data(self, point_cloud_msg):
        global FRAME
        data=[]
        for i in range(len(point_cloud_msg.points)):
            data.append([point_cloud_msg.points[i].x, point_cloud_msg.points[i].y, point_cloud_msg.points[i].z])
               

        ##
        point_dummy=[]
        for i in range(len(data)):
            Point_32=Point32()
            Point_32.x=data[i][0]
            Point_32.y=data[i][1]
            Point_32.z=data[i][2]
            point_dummy.append(Point_32)
        point_cloud_msg__ = PointCloud()
        point_cloud_msg__.header = point_cloud_msg.header
        point_cloud_msg__.points = point_dummy
        ##

        tree = KDTree(data)
        neighbors = tree.query_radius(data, r = THRESHOLD_RADIUS_KDTREE)
        centroids=[]
        bounding_box=[]
        x=0.0
        y=0.0
        z=0.0

        min_x=float('inf')
        min_y=float('inf')
        
        max_x=float('-inf')
        max_y=float('-inf')

        if len(centroids)==0:
            for i in range(len(neighbors[0])):
                x+=data[neighbors[0][i]][0]
                y+=data[neighbors[0][i]][1]
                z+=data[neighbors[0][i]][2]
                if min_x > data[neighbors[0][i]][0]:
                    min_x = data[neighbors[0][i]][0]
                if max_x < data[neighbors[0][i]][0]:
                    max_x = data[neighbors[0][i]][0]
                if min_y > data[neighbors[0][i]][1]:
                    min_y = data[neighbors[0][i]][1]
                if max_y > data[neighbors[0][i]][1]:
                    max_y = data[neighbors[0][i]][1]
            value_list=[x/len(neighbors[0]), y/len(neighbors[0]), z/len(neighbors[0])]
            centroids.append(value_list)
            bounding_box.append([min_x, min_y, max_x, max_y])

                
        for n in range(1,len(neighbors)):
            x=0.0
            y=0.0
            z=0.0

            min_x=float('inf')
            min_y=float('inf')
            
            max_x=float('-inf')
            max_y=float('-inf')
                        
            for i in range(len(neighbors[n])):
                x+=data[neighbors[n][i]][0]
                y+=data[neighbors[n][i]][1]
                z+=data[neighbors[n][i]][2]
                if min_x > data[neighbors[n][i]][0]:
                    min_x = data[neighbors[n][i]][0]
                if max_x < data[neighbors[n][i]][0]:
                    max_x = data[neighbors[n][i]][0]
                if min_y > data[neighbors[n][i]][1]:
                    min_y = data[neighbors[n][i]][1]
                if max_y > data[neighbors[n][i]][1]:
                    max_y = data[neighbors[n][i]][1]
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

            else:    
                centroids.append(centre)
                bounding_box.append([min_x, min_y, max_x, max_y])

                
        #static vs dynamic
        #print("FRAME = ", FRAME)
        #curr_frame_data = [centroids, bounding_box]
        #if FRAME ==1:
        #    self.first_frame_data = curr_frame_data
        #    # print("first_frame_data=",  self.first_frame_data)
        #elif FRAME ==2 and FRAME==3:
        #    #compare and find common points
        #    if FRAME ==2:
        #        self.common_data = self.first_frame_data
        #    print("second/third_frame_data=",  self.common_data)
        #    [self.common_data] = self.find_static_common_points(self.common_data, curr_frame_data)
        #else:
        #    data = self.dynamic_point_data(self.common_data, curr_frame_data)
#
        #centroids = data[0]
        #print("centroids=", centroids)

        #/subscribe to /frameid topic = FRAME
        
        self.curr_frame_data = [centroids, bounding_box]
        FRAME = 
        if FRAME ==1:
            self.first_frame_data = curr_frame_data
            # print("first_frame_data=",  self.first_frame_data)
        elif FRAME ==2 and FRAME==3:
            #compare and find common points
            if FRAME ==2:
                self.common_data = self.first_frame_data
            print("second/third_frame_data=",  self.common_data)
            [self.common_data] = self.find_static_common_points(self.common_data, curr_frame_data)
        else:
            data = self.dynamic_point_data(self.common_data, curr_frame_data)

        centroids = data[0]
        print("centroids=", centroids)

        #increment FRAME and publish to /frameid topic



        #giving out data
        data_cloud=[]
        
        for cen in centroids:
            centroids_Point=Point32()
            centroids_Point.x=cen[0]
            centroids_Point.y=cen[1]
            centroids_Point.z=cen[2] 
            data_cloud.append(centroids_Point)   
        centroids.clear()
        #print("num_neighbours= ", num_neighbors)
        point_cloud_msg_ = PointCloud()
        point_cloud_msg_.header = point_cloud_msg.header
        point_cloud_msg_.points = data_cloud
        return point_cloud_msg_,point_cloud_msg__
        
        
        
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

#class N1
class InitFrameNode(Node):
    def __init__(self):
        super().__init__('init_frame_node')
        self.first_frame_data=[]
        self.common_data =[]
        self.frameid_publisher = self.create_publisher(Int64, 'frame_id', 10)
    
    def publish_frameid(self):
        int64_msg = Int64()
        int64_msg.data = 1
        self.frameid_publisher.publish(int64_msg)


#init - publishes 1 to a topic /frameid
        
def main(args=None):
    rclpy.init(args=args)

    #create Onetime node N1
    initframe_node = InitFrameNode()
    initframe_node.publish_frameid()
    #destroynode
    print('node_1  start')
    node = LaserScanNode()
    initframe_node.destroy_node()
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