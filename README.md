# Pedestrian_tracking_using_Lidar
ROS2 project to track and count the number of pedestrians encountered in Lidar Point Cloud data

STARTING TASK CHECKLIST:
1. Explore Launch files and Bags in Ros2
2. Explore Rviz2
3. Understand which tracking algorithm to  use
4. Design the problem with at least 2 nodes

TASK for Oct 17, 2023- All 3 of us are going to check the following and keep on adding tutorial links:
1. Launch
   a. https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Launch-Main.html  - Managing Large project explains how to launch rviz from launch files to visualize
   b. https://design.ros2.org/articles/roslaunch.html 
2. Bags
   a. https://docs.ros.org/en/humble/Tutorials/Advanced/Recording-A-Bag-From-Your-Own-Node-Py.html - writing to bag
   b. https://mcap.dev/guides/python/ros2 - Reading and writing from bags
   c. https://roboticsbackend.com/ros2-bag-save-and-replay-topic-data/ - replaying topic data saved in bag file (only helpful for initial understanding)
4. Rviz2
   a. https://www.youtube.com/watch?v=yNd-ZqWKsBc - rviz2 but with gazebo. We can filter out the rviz2 portions -LOL watched the same one


## Visualizing ROS 2 Bag Data Using RViz2
<details>
    <summary>Rviz2_visualization</summary>

Follow these steps to visualize data from a ROS 2 bag file using RViz2:

1. Change to the directory where your ROS 2 bag file is located:

   ```bash
   cd /path/to/your/bag/folder
   source /opt/ros/humble/setup.bash

2. Launch RViz2:

   ```bash
   ros2 run rviz2 rviz2

3. In the RViz2 interface:

   ```bash
   Open the "Global Options" panel.
   Set the "Fixed frame" to "laser."

4. Add a LaserScan display:

   ```bash
   Click "Add" in the RViz2 interface.
   Select "LaserScan."
   Specify the "Topic" as /scan.

5. Open a new terminal in the same folder:

   ```bash

   source /opt/ros/humble/setup.bash

6. Play your ROS 2 bag file using the ros2 bag play command. Replace example9.db3 with the actual name of your bag file:

   ```bash


   while true; do ros2 bag play example9.db3 ; done

7. In the RViz2 interface, you can also reset the time to synchronize the time? I don't know exactly how it goes.

<!-- summary 아래 한칸 공백 두고 내용 삽입 -->

</details>

## How to setup a launch file
<details>
    <summary>Setup Launch file</summary>

Follow these steps to build a launch file

1. Change to the directory where your ROS 2 package file is located:

   ```bash
   cd /ros-folder/src/package_name
   mkdir launch
   touch launch_.py
  (should not be the same as 'launch' library)

2. Use the launch_.py in the files
3. Change the setup.py

   ```bash
   data_files=[
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    ('share/' + package_name, glob.glob('launch/*'))
   ],

   entry_points={
        'console_scripts': [
             'your_command = project_3.your_python_file_name:main',
             'your_command_2 = project_3.your_python_file_name:main_2'
        ],
    },

4. Colcon build on src folder

   ```bash
   colcon build

5. Launch the launch file (you can check it on Rviz2)

   ```bash
   ros2 launch project_3 launch_.py bag_in:=[your_folder]/example9.db3 bag_out:=[your_estimated_new_folder]
<!-- summary 아래 한칸 공백 두고 내용 삽입 -->

</details>
## Data Type Descriptions given in the Bag files:
### ros2 interface show tf2_msgs/msg/TFMessage
geometry_msgs/TransformStamped[] transforms
	#
	#
	std_msgs/Header header
		builtin_interfaces/Time stamp
			int32 sec
			uint32 nanosec
		string frame_id
	string child_frame_id
	Transform transform
		Vector3 translation
			float64 x
			float64 y
			float64 z
		Quaternion rotation
			float64 x 0
			float64 y 0
			float64 z 0
			float64 w 1
   
### ros2 interface show sensor_msgs/msg/LaserScan
### Single scan from a planar laser range-finder. If you have another ranging device with different behavior (e.g. a sonar array), please find or create a different message, since applications will make fairly laser-specific assumptions about this data

std_msgs/Header header # timestamp in the header is the acquisition time of
	builtin_interfaces/Time stamp
		int32 sec
		uint32 nanosec
	string frame_id
                             # the first ray in the scan.
                             #
                             # in frame frame_id, angles are measured around
                             # the positive Z axis (counterclockwise, if Z is up)
                             # with zero angle being forward along the x axis

float32 angle_min            # start angle of the scan [rad]
float32 angle_max            # end angle of the scan [rad]
float32 angle_increment      # angular distance between measurements [rad]

float32 time_increment       # time between measurements [seconds] - if your scanner
                             # is moving, this will be used in interpolating position
                             # of 3d points
float32 scan_time            # time between scans [seconds]

float32 range_min            # minimum range value [m]
float32 range_max            # maximum range value [m]

float32[] ranges             # range data [m]

## Published Topics
ros2 interface show sensor_msgs/msg/PointCloud
### This message holds a collection of 3d points, plus optional additional information about each point.

### Time of sensor data acquisition, coordinate frame ID.
std_msgs/Header header
	builtin_interfaces/Time stamp
		int32 sec
		uint32 nanosec
	string frame_id

### Array of 3d points. Each Point32 should be interpreted as a 3d point in the frame given in the header.
geometry_msgs/Point32[] points
	#
	#
	float32 x
	float32 y
	float32 z

### Each channel should have the same number of elements as points array,and the data in each channel should correspond 1:1 with each point. Channel names in common practice are listed in ChannelFloat32.msg.
ChannelFloat32[] channels
	#
	string name

                             # (Note: values < range_min or > range_max should be discarded)
float32[] intensities        # intensity data [device-specific units].  If your
                             # device does not provide intensities, please leave

