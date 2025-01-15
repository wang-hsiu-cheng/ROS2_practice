from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch_ros.actions import Node
from launch.event_handlers import OnExecutionComplete
  
def generate_launch_description():
     
    turtlesim1 = Node(
        package="turtlesim",
        executable="turtlesim_node",
        name='turtlesim1'
    )
 
    turtlesim2 = Node(
        package="turtlesim",
        executable="turtlesim_node",
        name='turtlesim2'
    )
 
    return LaunchDescription([
        RegisterEventHandler(
            event_handler=OnExecutionComplete(
                target_action=turtlesim1,
                on_completion=[turtlesim2],
            )
        ),
        turtlesim1
    ])