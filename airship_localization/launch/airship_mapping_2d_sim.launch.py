from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import Shutdown

def generate_launch_description():
    ## ***** Launch arguments *****
    bag_filename_arg = DeclareLaunchArgument('bag_filename')

    ## ***** Nodes *****
    backpack_2d_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(FindPackageShare('airship_localization').find('airship_localization') + '/launch/2d_mapping_sim.launch.py'),
        launch_arguments = {'use_sim_time': 'True'}.items()
        )

    rviz_node = Node(
        package = 'rviz2',
        executable = 'rviz2',
        on_exit = Shutdown(),
        arguments = ['-d', FindPackageShare('airship_localization').find('airship_localization') + '/configuration_files/cartographer_2d.rviz'],
        parameters = [{'use_sim_time': True}],
    )

    ros2_bag_play_cmd = ExecuteProcess(
        cmd = ['ros2', 'bag', 'play', LaunchConfiguration('bag_filename'), '--clock'],
        name = 'rosbag_play',
    )

    return LaunchDescription([
        # Launch arguments
        bag_filename_arg,
        # Nodes
        backpack_2d_launch,
        rviz_node,
        ros2_bag_play_cmd
    ])
