#Authors: Shilpa Mukhopadhyay, Amrita Mohandas, Junsuk Kim
#UIN: 433003777, 534002383, 334003508
#Instructor: Dr. Jason O'Kane
#TA: Aaron Kingery 
#Course ID: CSCE 752
#Course Title: Robotics and Spatial Intelligence
#Project 4_a
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import numpy as np
from geometry_msgs.msg import Twist,Vector3
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from math import sin, cos, pi
import math
from rclpy.qos import QoSProfile
from sensor_msgs.msg import JointState

class Simulator_node(Node):
    def __init__(self):
        super().__init__('robot_simulator')
        self.declare_parameter('initial_x', 0.0)
        self.declare_parameter('initial_y', 0.0)
        self.declare_parameter('initial_theta', 0.0)
        self.declare_parameter('wheel_radius', 0.1)
        self.declare_parameter('base_width', 0.2)
        self.declare_parameter('update_rate', 10.0)
        self.declare_parameter('error_rate', 0.1)

        self.tf_broadcaster = TransformBroadcaster(self)

        self.current_x = self.get_parameter('initial_x').value
        self.current_y = self.get_parameter('initial_y').value
        self.current_theta = self.get_parameter('initial_theta').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.base_width = self.get_parameter('base_width').value
        self.update_rate = self.get_parameter('update_rate').value
        self.error_rate = self.get_parameter('error_rate').value

        self.vl_subscription = self.create_subscription(Float64, '/vl', self.vl_callback, 10)
        self.vr_subscription = self.create_subscription(Float64, '/vr', self.vr_callback, 10)

        self.timer = self.create_timer(1.0 / self.update_rate, self.update_pose)
        self.vl=0
        self.vr=0

    def vl_callback(self, msg):
        # Left wheel velocity callback
        self.vl = msg.data

    def vr_callback(self, msg):
        # Right wheel velocity callback
        self.vr = msg.data

    def update_pose(self):
        # Calculate linear and angular velocity
        v = (self.vl + self.vr) / 2.0
        w = (self.vr - self.vl) / self.base_width

        # Update pose with errors
        v += v * self.error_rate
        w += w * self.error_rate

        # Update position and orientation
        self.current_x += v * math.cos(self.current_theta)
        self.current_y += v * math.sin(self.current_theta)
        self.current_theta += w

        # Broadcast the transform from world to base_link
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.current_x
        t.transform.translation.y = self.current_y
        t.transform.rotation.z = math.sin(self.current_theta / 2)
        t.transform.rotation.w = math.cos(self.current_theta / 2)
        self.tf_broadcaster.sendTransform(t)
    

class Velocity_translator_node(Node):
    def __init__(self):
        super().__init__('Velocity_transfer_node')
        # Publish intermediate topic and publish 'person_location' and 'person_count' topic.
        self.twist_subscriber = self.create_subscription(Twist, 'cmd_vel', self.twist_callback, 50)
        
        # FOr each 1 cmd_vel -> 1 vl + 1 vr
        self.left_publisher = self.create_publisher(Float64, 'vl', 20)
        self.right_publisher = self.create_publisher(Float64, 'vr', 20)

    # only job -> translate linear.x and angular.x
    
    # A callback function for subscribing and publishing.
    def twist_callback(self, msg):
        linear_x = msg.linear.x
        angular_z = msg.angular.z

        # Calculate wheel velocities from linear and angular components
        # You can implement your own logic for conversion here
        # For example, you can use a differential drive kinematic model
        vr = linear_x + angular_z
        vl = linear_x - angular_z

        vr_msg = Float64()
        vr_msg.data = vr
        vl_msg = Float64()
        vl_msg.data = vl

        self.left_publisher.publish(vl_msg)
        self.right_publisher.publish(vr_msg)

def main(args=None):
    rclpy.init(args=args)
    print('node_1 start')
    node = Simulator_node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

def main_2(args=None):
    rclpy.init(args=args)
    print('node_2 start')
    node_2=Velocity_translator_node()
    rclpy.spin(node_2)
    node_2.destroy_node()
    rclpy.shutdown()
