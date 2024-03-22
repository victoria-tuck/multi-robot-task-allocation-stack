import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PythonExpression

def generate_launch_description():
    
    social_navigation_dir = get_package_share_directory('social_navigation_py')    
    social_navigation_config_dir = os.path.join( get_package_share_directory('social_navigation'), 'configs')
    
    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')
    
    human_sfm = Node(
        package='social_navigation_py',
        executable='human_sfm',
        name = 'human_sfm',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'num_humans': 10,
            'timer_period': 0.05,
            'human_config_file': os.path.join(
            social_navigation_config_dir, 'humans_waypoint_config2.yaml')
                     }]
    )
    
    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(human_sfm)
    return ld
