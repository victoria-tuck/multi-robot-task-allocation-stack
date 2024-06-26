import os
import sys

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
    
    arguments = parse_arguments(sys.argv)

    goal_setter = Node(
        package='social_navigation_py',
        executable='goal_setter',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time}],
        arguments=arguments
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(goal_setter)
    return ld


def parse_arguments(argv):
    args = []
    for arg in argv:
        if ":=" in arg:
            parsed_arg = arg.split(':')
            print(parsed_arg)
            args.append(f"-{parsed_arg[0]}")
            args.append(parsed_arg[1][1:])
    return args
