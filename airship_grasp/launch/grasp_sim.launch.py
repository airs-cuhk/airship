import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import launch_ros.actions

def generate_launch_description():
    airship_grasp_arg = DeclareLaunchArgument(
                            'config',
                            default_value=os.path.join(get_package_share_directory('airship_grasp'), 'config/airship_grasp_sim_config.yaml'),
                            description='Path to the config file'
                        )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Whether to use simulation time'
    )
    
    airship_grasp_node = launch_ros.actions.Node(
        # namespace= "airship", 
        package='airship_grasp', 
        executable='grasp_server', 
        output='screen',
        parameters=[{'config': LaunchConfiguration('config')},
                    {'use_isaac_sim': LaunchConfiguration('use_isaac_sim')},
                    {'use_sim_time': LaunchConfiguration('use_sim_time')}]
        )

    return LaunchDescription([
        airship_grasp_arg,
        use_sim_time_arg,
        airship_grasp_node,
    ])
