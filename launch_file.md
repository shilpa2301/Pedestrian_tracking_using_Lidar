Follow these steps to build a launch file

1. Change to the directory where your ROS 2 package file is located:

   ```bash
   cd /ros-folder/src/package_name
   mkdir launch
   touch launch_.py
  (should not be the same as 'launch' library)

2. launch_.py
   ```bash
   from launch import LaunchDescription
   from launch_ros.actions import Node
   from launch.actions import ExecuteProcess, DeclareLaunchArgument, LogInfo, RegisterEventHandler, EmitEvent
   from launch.conditions import IfCondition
   from launch.event_handlers import OnProcessExit
   from launch.events import Shutdown
   from launch.substitutions import LaunchConfiguration
   
   def generate_launch_description():
       # Declare launch argument 'bag_in'
       arg_in_value = LaunchConfiguration('bag_in')
       arg_out_value=LaunchConfiguration('bag_out')
       arg_bag_in = DeclareLaunchArgument('bag_in', default_value='[your_folder]/example9.db3', description='Input bag file path')
       
       # Declare launch argument 'bag_out'
       arg_bag_out = DeclareLaunchArgument('bag_out', default_value='[change_it_to_your_expected_new_folder]', description='Output bag file path')
       
   
       
       # Define a Node that uses 'bag_in' and 'bag_out' arguments
       bag_play = ExecuteProcess(
           cmd=['ros2', 'bag', 'play',arg_in_value],
           output='screen',
       )
       
       node = Node(
           package='project_3',
           executable='do_it',
           name='LaserScanToPointCloudNode',
           output='screen',
       )
   
       # Execute an additional process (ros2 bag record) using ExecuteProcess
       bag_record = ExecuteProcess(
           cmd=['ros2', 'bag', 'record', '-o', arg_out_value,'topic_name', '/point_cloud'],
           output='screen',
       )
   
       # Define an event handler that initiates an overall shutdown when 'node' exits
       event_handler = OnProcessExit(target_action=node, on_exit=[EmitEvent(event=Shutdown())])
   
       # Register the event handler
       terminate_at_end = RegisterEventHandler(event_handler)
       ld = LaunchDescription([ arg_bag_in,arg_bag_out,bag_play,bag_record,node,terminate_at_end])
       return  ld
   


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
   ros2 launch project_3 launch_.py bag_in:=[your_folder]/example9.db3
