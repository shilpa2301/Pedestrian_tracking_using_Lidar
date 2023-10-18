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
   Open the "Global Status" panel.
   Set the "Fixed frame" to "world."

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

   ros2 bag play example9.db3

7. In the RViz2 interface, you can also reset the time to start visualizing data from the beginning.
