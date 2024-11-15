from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

# Function to generate launch description


def generate_launch_description():
    # Declare frequency argument
    freq_arg = DeclareLaunchArgument(
        'freq',
        default_value='1.0',  # Default frequency if none is provided
        description='Frequency for the publisher node in Hz'
    )
    # Declare test duration argument

    test_duration = DeclareLaunchArgument(
        'test_duration',
        default_value='5.0',
        description="Max length of test in seconds"
    )
    result_file_arg = DeclareLaunchArgument(
        'result_file',
        default_value='/home/keyur22/enpm700/ROS2_tut/ros2_ws/build/beginner_tutorials/test_results/beginner_tutorials/Level2_Integration_Test.xml',
        description='File path where test results are saved.'
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
    lvl2_integration_test = Node(
            package='beginner_tutorials',
            executable='level2_integration_test',
            name='integration_test',
            parameters=[{
                'test_duration': LaunchConfiguration('test_duration')

            }],
            # arguments=[LaunchConfiguration('result_file')],
             output='screen',
            # Set 'required=True' to terminate the launch when this node finishes
            on_exit=[Shutdown()]
    )
    # return launch description
    return LaunchDescription([
        freq_arg,
        test_duration,
        # lvl2_integration_test,
        # result_file_arg,
        minimal_publisher_node
        # minimal_subscriber_node
    ])
