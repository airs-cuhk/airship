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
                            default_value=os.path.join(get_package_share_directory('airship_grasp'), 'config/airship_grasp_config.yaml'),
                            description='Path to the config file'
                        )

    camera_node  =  IncludeLaunchDescription(
                        PythonLaunchDescriptionSource([os.path.join(
                            get_package_share_directory('realsense2_camera'), 'launch', 'rs_launch.py')]),
                        launch_arguments={
                            'depth_module.depth_profile': '1280x720x30',
                            'rgb_camera.color_profile': '1280x720x30',
                            'align_depth.enable': 'true'
                        }.items(),
                        )
    
    airship_grasp_node = launch_ros.actions.Node(
        # namespace= "airship", 
        package='airship_grasp', 
        executable='grasp_server', 
        output='screen',
        parameters=[{'config': LaunchConfiguration('config')},
                    {'use_isaac': LaunchConfiguration('use_isaac')}]
        )

    return LaunchDescription([
        airship_grasp_arg,
        camera_node,
        airship_grasp_node,
    ])
