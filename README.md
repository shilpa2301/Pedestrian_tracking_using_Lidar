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
[Rviz2_visualization](.rviz2_visualize.md)

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

