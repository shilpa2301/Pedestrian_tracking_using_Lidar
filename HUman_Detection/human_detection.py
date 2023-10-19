import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud
from geometry_msgs.msg import Point32
import math
from std_msgs.msg import Float32
from example_interfaces.msg import Int64

from project3.lidar_custom import *
from project3.pub_loc_count import *
from project3.scan_subscriber import *

class Human_Detection( LaserScanToPointCloudNode, PublishPersonLocationCount, LaserScanSubscriber):
    def __init__(self):
        super().__init__('intruder_detection')

    #def get_data(self):
        


def main(args=None):
    rclpy.init(args=args)
    node = Human_Detection()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()