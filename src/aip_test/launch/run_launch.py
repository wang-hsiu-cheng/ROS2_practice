from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package = 'turtlesim',
            executable = 'turtlesim_node',
            name = 'turtlesim_node'
        ),
        TimerAction(
            period = 1.0,
            actions = [Node(
                package = 'aip_test',
                executable = 'talker',
                name = 'talker'
            )],
        ),
    ])
