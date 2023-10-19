
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
