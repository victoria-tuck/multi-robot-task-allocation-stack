import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PythonExpression

def generate_launch_description():
    
    social_navigation_dir = get_package_share_directory('social_navigation')    
    
    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')
    
    num_humans = LaunchConfiguration('num_humans')
    declare_num_humans_cmd = DeclareLaunchArgument(
        'num_humans',
        default_value='10',
        description='Number of humans')
    
    get_human_states = Node(
        package='social_navigation',
        executable='get_human_states',
        name = 'get_human_states',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'num_humans': num_humans,
            'update_frequency': 20.0
                     }]
    )
    
    robot_closest_obstacle_sector = Node(
        package='social_navigation',
        executable='robot_closest_obstacle_sector',
        name = 'robot_closest_obstacle_sector',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'num_humans': num_humans
            }]
    )   

    human_closest_obstacle_sector = Node(
        package='social_navigation',
        executable='human_closest_obstacle_sector',
        name = 'human_closest_obstacle_sector',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'num_humans': num_humans
            }]
    )   
    
    
    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_num_humans_cmd)
    ld.add_action(get_human_states)
    ld.add_action(robot_closest_obstacle_sector)
    ld.add_action(human_closest_obstacle_sector)
    return ld
