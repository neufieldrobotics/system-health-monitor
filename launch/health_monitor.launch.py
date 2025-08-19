from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    period = LaunchConfiguration('period_sec', default='2.0')

    return LaunchDescription([
        DeclareLaunchArgument('period_sec', default_value='2.0',
                              description='Diagnostics publish period (seconds)'),

        Node(
            package='system_health_monitor',
            executable='computer_monitor',
            name='computer_monitor',
            output='screen',
            parameters=[{'period_sec': period}],
        ),

        Node(
            package='system_health_monitor',
            executable='node_monitor',
            name='node_monitor',
            output='screen',
            parameters=[{'period_sec': period}],
        ),

        # We don't want image monitoring -- too expensive for little gain
        # Node(
        #     package='system_health_monitor',
        #     executable='image_monitor',
        #     name='image_monitor',
        #     output='screen',
        #     parameters=[{'period_sec': period}],
        # ),
    ])