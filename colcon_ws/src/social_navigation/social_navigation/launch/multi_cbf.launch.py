import os
import sys
import yaml

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
    config_file = os.path.join(social_navigation_config_dir, 'case_config_6_withQ.yaml')
    print(f"Config file: {config_file}")
    
    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')
    
    with open(config_file, 'r') as file:
        config = yaml.safe_load(file)
    print(f"Config: {config}")
    
    agent_names = list(config["agents"].keys())
    print(f"Agent names: {agent_names}")
    if len(agent_names) > 1:
        other_agents = {agent_names[i]: agent_names[:i] + agent_names[i+1:] for i in range(len(agent_names))} # Make sure this can work for one agent
    else:
        other_agents = {agent_names[0]: ['robot2']}

    controllers = [Node(
        package='social_navigation_py',
        executable='nav2_cbf',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_name': agent,
            'robot_list': other_agents[agent]}]
    ) for agent in agent_names]

    ld = LaunchDescription()
    for controller in controllers:
        ld.add_action(declare_use_sim_time_cmd)
        ld.add_action(controller)
    return ld
