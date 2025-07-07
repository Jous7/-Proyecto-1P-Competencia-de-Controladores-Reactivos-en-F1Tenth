from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('speed', default_value='2.0'),
        DeclareLaunchArgument('gain', default_value='0.5'),

        Node(
            package='f1tenth_gap',
            executable='follow_gap_node',
            name='follow_gap_node',
            output='screen',
            parameters=[{
                'speed_base': LaunchConfiguration('speed'),
                'steering_gain': LaunchConfiguration('gain')
            }]
        )
    ])
