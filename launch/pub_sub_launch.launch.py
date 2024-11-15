import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, TimerAction, OpaqueFunction
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import ExecuteProcess
from launch.conditions import IfCondition


def launch_setup(context, *args, **kwargs):
    
    # Define the duration for which to record the bag (~15 seconds)
    record_duration = 15.0  # seconds

    # Set up the bag recording command
    record_bag = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-a'],
        condition=IfCondition(LaunchConfiguration('enable_bag_record')),
        output='screen',
        cwd=['src/beginner_tutorials/results/recorded_bag']
    )

    # Timer to stop the recording after ~15 seconds
    stop_recording = ExecuteProcess(
        cmd=['pkill', '-f', 'ros2 bag record'],
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_bag_record'))
    )

    return [record_bag, TimerAction(period=record_duration, actions=[stop_recording])]

# Function to generate launch description
def generate_launch_description():
    # Declare frequency argument
    freq_arg = DeclareLaunchArgument(
        'freq',
        default_value='1.0',  # Default frequency if none is provided
        description='Frequency for the publisher node in Hz'
    )
    # Define the enable_bag_record argument to toggle bag recording
    enable_bag_record = DeclareLaunchArgument(
        'enable_bag_record',
        default_value='false',
        description='Flag to enable or disable bag recording'
    )
    # Define the duration for which to record the bag (~15 seconds)
    record_duration = 15  # seconds
    # Set up the bag recording command
    record_bag = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-a'],
        # condition=LaunchConfiguration('enable_bag_record').to_bool() == True,
        output='screen'
    )
    # Timer to stop the recording after ~15 seconds
    stop_recording = ExecuteProcess(
        cmd=['pkill', '-f', 'ros2 bag record'],
        output='screen',
        # condition=LaunchConfiguration('enable_bag_record').to_bool() == True
    )

    # create handle for publisher node
    minimal_publisher_node = Node(
            package='beginner_tutorials',
            executable='talker',
            name='my_publisher',
            output='screen',
            parameters=[{'freq': LaunchConfiguration('freq')}],
        )
    # create handle for subscriber node
    minimal_subscriber_node = Node(
            package='beginner_tutorials',
            executable='listener',
            name='my_subscriber',
            output='screen',
        )
    
    # return launch description
    return LaunchDescription([
        freq_arg,
        enable_bag_record,
        minimal_publisher_node,
        minimal_subscriber_node,
        OpaqueFunction(function=launch_setup)
    ])
