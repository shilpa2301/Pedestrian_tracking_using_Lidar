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
    
    node_1 = Node(
        package='project_3',
        executable='<your_command>',
        name='LaserScanNode',
        output='screen',
    )
    node_2 = Node(
        package='project_3',
        executable='<your_command_2>',
        name='PointCloudNode',
        output='screen',
    )
    # Execute an additional process (ros2 bag record) using ExecuteProcess
    bag_record = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-o', arg_out_value,'topic_name', '/point_cloud'],
        output='screen',
    )

    # Define an event handler that initiates an overall shutdown when 'node' exits
    event_handler = OnProcessExit(target_action=bag_play, on_exit=[EmitEvent(event=Shutdown())])

    # Register the event handler
    terminate_at_end = RegisterEventHandler(event_handler)
    ld = LaunchDescription([ arg_bag_in,arg_bag_out,bag_play,bag_record,node_1,node_2,terminate_at_end])
    return  ld

