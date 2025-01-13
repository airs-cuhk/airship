import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node

def generate_launch_description():
    # Get URDF path.
    urdf_file = os.path.join(
        get_package_share_directory('airship_description'),
        'urdf',
        'airsbot2.urdf'
    )

    # Set robot_state_publisher node
    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command([
                "xacro ", urdf_file, " "
            ])
        }]
    )

    # Creat LaunchDescription
    return LaunchDescription([
        start_robot_state_publisher_cmd,
    ])
