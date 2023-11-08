from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, DeclareLaunchArgument,RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os
# The load_disc_robot method reads a file that describes a disc-shaped robot
# and returns a dictionary describing the properties of that robot.

import yaml

def load_disc_robot(file_name):
    with open(file_name) as f:
        robot = yaml.safe_load(f)
    robot['urdf'] = disc_robot_urdf(robot)
    return robot

def disc_robot_urdf(robot):
    radius = robot['body']['radius']
    height = robot['body']['height']

    return f"""<?xml version="1.0"?>
                  <robot name="disc">
                      <material name="light_blue">
                          <color rgba="0.5 0.5 1 1"/>
                      </material>
                      <material name="dark_blue">
                          <color rgba="0.1 0.1 1 1"/>
                      </material>
                      <material name="dark_red">
                          <color rgba="1 0.1 0.1 1"/>
                      </material>
                      <link name="base_link">
                          <visual>
                              <geometry>
                                  <cylinder length="{height}" radius="{radius}"/>
                              </geometry>
                              <material name="light_blue"/>
                          </visual>
                      </link>
                      <link name="heading_box">
                          <visual>
                              <geometry>
                                  <box size="{0.9*radius} {0.2*radius} {1.2*height}"/>
                              </geometry>
                              <material name="dark_blue"/>
                          </visual>
                      </link>
                      <link name="laser" />
                      <joint name="base_to_heading_box" type="fixed">
                          <parent link="base_link"/>
                          <child link="heading_box"/>
                          <origin xyz='{0.45*radius} 0.0 0.0'/>
                      </joint>
                      <joint name="base_to_laser" type="fixed">
                          <parent link="base_link"/>
                          <child link="laser"/>
                          <origin xyz="{0.5*radius} 0.0 0.0"/>
                      </joint>
                  </robot>
                  """


def generate_launch_description():
    urdf_file_name = 'model/normal_robot.xml'
    urdf = os.path.join(
        '/home/john2204/ros2_ws/src/project_4_a/',
        urdf_file_name)
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()
    node_state=Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}],
            arguments=[urdf],
    )
    
    
    node_1 = Node(
        package='project_4_a',
        executable='do_it',
        name='Simulator_node',
        output='screen',
        
    )
    node_2 = Node(
        package='project_4_a',
        executable='do_it_2',
        name='Velocity_translator_node',
        output='screen',
    )
    ld = LaunchDescription([ node_state,node_2,node_1])
    return  ld
