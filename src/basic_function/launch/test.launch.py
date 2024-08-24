import launch_ros.actions

from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
# for including other launch file
from launch.actions import IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
# for DeclareLaunchArgument
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("slam_ros2"), "rviz", "view_robot.rviz"]
    )
    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='turtlebot',
        description='Name of the robot'
    )
    return LaunchDescription([
        launch_ros.actions.SetParameter(
            name='use_sim_time', value=True
        ),
        Node(
            package = 'turtlesim',
            executable = 'turtlesim_node',
            name = LaunchConfiguration('robot_name')
        ),
        Node1(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="log",
            arguments=["-d", rviz_config_file],
        ),
        LogInfo(
            msg=LaunchConfiguration('robot_name')
        ),
        TimerAction(
            period = 1.0,
            actions = [Node(
                package = 'aip_test',
                executable = 'talker',
                name = 'talker'
                arguments = ['-topic', 'robot_description', 
                        '-entity', 'ur5',
                        '-z', '0.1']
                parameters=['../yaml/example.yaml']
                
            )],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare("basic_function"), '/launch', '/turtlesim_launch.xml'])
        )
    ])
