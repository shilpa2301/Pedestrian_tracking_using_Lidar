from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, DeclareLaunchArgument,RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    
    arg_in_value = LaunchConfiguration('bag_in')
    arg_out_value=LaunchConfiguration('bag_out')
    # Declare launch argument 'bag_in'
    arg_bag_in = DeclareLaunchArgument('bag_in', default_value='/home/john2204/ros2_ws/bags/example9/example9.db3', description='Input bag file path')
    # Declare launch argument 'bag_out'
    arg_bag_out = DeclareLaunchArgument('bag_out', default_value='/home/john2204/ros2_ws/bags/example9/example', description='Output bag file path')    
    # Define a Node that uses 'bag_in' and 'bag_out' arguments
    bag_play = ExecuteProcess(
        cmd=['ros2', 'bag', 'play',arg_in_value],
        output='screen',
    )
    
    node_1 = Node(
        package='project_3',
        executable='do_it',
        name='LaserScanNode',
        output='screen',
    )
    node_2 = Node(
        package='project_3',
        executable='do_it_2',
        name='PointCloudNode',
        output='screen',
    )
    # Execute an additional process (ros2 bag record) using ExecuteProcess
    bag_record = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-a','-o', arg_out_value],
        output='screen',
    )
    event_handler = OnProcessExit(target_action=bag_play, on_exit=[EmitEvent(event=Shutdown())])
    terminate_at_end = RegisterEventHandler(event_handler)
    ld = LaunchDescription([ arg_bag_in,arg_bag_out,bag_play,bag_record,node_1,node_2,terminate_at_end])
    return  ld
