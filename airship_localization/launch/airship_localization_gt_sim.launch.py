from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import Shutdown
import os

def generate_launch_description():
    ## ***** Launch arguments *****

    ## ***** File paths ******
    pkg_share = FindPackageShare('airship_description').find('airship_description')
    urdf_dir = os.path.join(pkg_share, 'urdf')
    urdf_file = os.path.join(urdf_dir, 'airsbot2.urdf')
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    ## ***** Nodes *****
    robot_state_publisher_node = Node(
        package = 'robot_state_publisher',
        executable = 'robot_state_publisher',
        parameters=[
            {'robot_description': robot_desc},
            {'use_sim_time': True}],
        output = 'screen'
        )
    
    isacc_convert_node = Node(
        package = 'airship_localization',
        executable = 'isaac_conversion_node'
    )

    isaac_odom_to_tf_node = Node(
        package='airship_localization',
        executable='isaac_odom_to_tf_node',
    )

    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_odom_to_base',
        parameters=[{'use_sim_time': True}],
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_footprint'],
        output='screen'
    )

    return LaunchDescription([
        # Nodes
        robot_state_publisher_node,
        isacc_convert_node,
        isaac_odom_to_tf_node,
        static_tf_node
    ])
