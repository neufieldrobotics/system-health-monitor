from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    period = LaunchConfiguration('period_sec', default='2.0')

    return LaunchDescription([
        DeclareLaunchArgument('period_sec', default_value='2.0',
                              description='Diagnostics publish period (seconds)'),

        # Node: computer-level diagnostics publisher (wrapper)
        Node(
            package='neuroam_monitoring',     # <-- your package name
            executable='computer_monitor_node',  # <-- see setup.py entry_points below
            name='computer_monitor',
            output='screen',
            parameters=[{'period_sec': period}],
        ),

        # Node: graph/required-topic monitor (your existing node)
        Node(
            package='neuroam_monitoring',     # <-- same package (or change accordingly)
            executable='node_monitor',        # <-- see setup.py entry_points below
            name='node_monitor_diag',
            output='screen',
            # If you add a param later (e.g., period), pass it here as well
            # parameters=[{'period_sec': period}],
        ),
    ])
