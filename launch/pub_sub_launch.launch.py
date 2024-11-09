from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

# Function to generate launch description
def generate_launch_description():
    
    # Declare frequency argument
    freq_arg = DeclareLaunchArgument(
        'freq',
        default_value='5.0',  # Default frequency if none is provided
        description='Frequency for the publisher node in Hz'
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
        minimal_publisher_node,
        minimal_subscriber_node
    ])