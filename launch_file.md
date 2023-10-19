Follow these steps to build a launch file

1. Change to the directory where your ROS 2 package file is located:

   ```bash
   cd /ros-folder/src/package_name
   mkdir launch
   touch launch_.py
  (should not be the same as 'launch' library)

2. launch_.py
   [launch_file](launch_.py)


2. Change the setup.py

   ```bash
   data_files=[
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    ('share/' + package_name, glob.glob('launch/*'))
   ],

3. Colcon build on src folder

   ```bash
   colcon build

4. Launch the launch file (you can check it on Rviz2)

   ```bash
   ros2 launch project_3 launch_.py bag_in:=[your_folder]/example9.db3 bag_out:=[your_estimated_new_folder]
