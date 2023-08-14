import os
from os import environ
from os import pathsep
from scripts import GazeboRosPaths
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, DeclareLaunchArgument, ExecuteProcess, Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution, LaunchConfiguration, PythonExpression, EnvironmentVariable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    env_cmd1 = SetEnvironmentVariable(name='TURTLEBOT3_MODEL', value='waffle')
    env_cmd2 = SetEnvironmentVariable(name='GAZEBO_MODEL_PATH', value='$GAZEBO_MODEL_PATH:/opt/ros/galactic/share/turtlebot3_gazebo/models')
    env_cmd3 = SetEnvironmentVariable(name='GAZEBO_MODEL_PATH', value='$GAZEBO_MODEL_PATH:/home/colcon_ws/src/aws-robomaker-hospital-world/ignition_models')
    social_navigation_dir = get_package_share_directory('social_navigation')

    world_file = os.path.join(social_navigation_dir, 'worlds', 'waffle_aws_hospital.world'),

    my_gazebo_models = PathJoinSubstitution([
        FindPackageShare('gazebo_sfm_plugin'),
        'models',
    ])


    # src_gazebo_cmd = [
    #     'source ',
    #     '/usr/share/gazebo/setup.sh'
    # ]
    # ExecuteProcess(
    #     cmd=src_gazebo_cmd,
    #     #output='screen',
    #     shell=True
    # )
    # print('gazebo_source:', src_gazebo_cmd)

    
    model, plugin, media = GazeboRosPaths.get_paths()
    #print('model:', model)

    if 'GAZEBO_MODEL_PATH' in environ:
        model += pathsep+environ['GAZEBO_MODEL_PATH']
    if 'GAZEBO_PLUGIN_PATH' in environ:
        plugin += pathsep+environ['GAZEBO_PLUGIN_PATH']
    if 'GAZEBO_RESOURCE_PATH' in environ:
        media += pathsep+environ['GAZEBO_RESOURCE_PATH']

    env = {
        'GAZEBO_MODEL_PATH': model,
        'GAZEBO_PLUGIN_PATH': plugin,
        'GAZEBO_RESOURCE_PATH': media
    }
    print('env:', env)

    
    world_path = PathJoinSubstitution([
        FindPackageShare('gazebo_sfm_plugin'),
        'worlds',
        'cafe3.world'
    ])
    
    # world_path = world_file

    gzserver_cmd = [
        'gzserver ',
        #'-u ', #to start paused
        # Pass through arguments to gzserver
        LaunchConfiguration('world'), world_path, 
        _boolean_command('verbose'), '',
    ]

    gzclient_cmd = [
        'gzclient',
        _boolean_command('verbose'), ' ',
    ]

    return LaunchDescription([
        
        env_cmd1,
        env_cmd2,
        env_cmd3,

        SetEnvironmentVariable(
            name='GAZEBO_MODEL_PATH', 
            value=[EnvironmentVariable('GAZEBO_MODEL_PATH'), my_gazebo_models]
        ),
        SetEnvironmentVariable(
            name='GAZEBO_RESOURCE_PATH', 
            value=[EnvironmentVariable('GAZEBO_RESOURCE_PATH'), my_gazebo_models]
        ),
        SetEnvironmentVariable(
            name='GAZEBO_PLUGIN_PATH', 
            value=[EnvironmentVariable('GAZEBO_PLUGIN_PATH'), plugin]
        ),

        DeclareLaunchArgument(
            'world', default_value='',
                     #'/home/kenny/ros2_ws/src/human_nav_gazebo_plugin/worlds/cafe2.world',
            description='Specify world file name'
        ),
        DeclareLaunchArgument(
            'verbose', default_value='true',
            description='Set "true" to increase messages written to terminal.'
        ),


        ExecuteProcess(
            cmd=gzserver_cmd,
            output='screen',
            #additional_env=env,
            shell=True,
            on_exit=Shutdown(),
            #condition=IfCondition(LaunchConfiguration('server_required')),
        ),

        ExecuteProcess(
            cmd=gzclient_cmd,
            output='screen',
            #additional_env=env,
            shell=True,
            on_exit=Shutdown(),
            #condition=IfCondition(LaunchConfiguration('server_required')),
        ),
    ])


# Add boolean commands if true
def _boolean_command(arg):
    cmd = ['"--', arg, '" if "true" == "', LaunchConfiguration(arg), '" else ""']
    py_cmd = PythonExpression(cmd)
    return py_cmd
