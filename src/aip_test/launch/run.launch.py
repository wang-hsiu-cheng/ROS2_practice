from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    simulater = Node(
        package='turtlesim',
        executable='turtlesim_node',
        name='turtlesim_node',
        output='screen'
    )

    controller = Node(
        package='aip_test',
        executable='talker',
        name='talker',
        output='screen'
    )

    return LaunchDescription([
        node_1,
        node_2.on_exit(node_1)
    ])
