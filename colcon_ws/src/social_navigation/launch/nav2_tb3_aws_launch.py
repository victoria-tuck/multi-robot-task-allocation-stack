import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable

def generate_launch_description():
    
    # Set environment variable
    env_cmd1 = SetEnvironmentVariable(name='TURTLEBOT3_MODEL', value='waffle')
    env_cmd2 = SetEnvironmentVariable(name='GAZEBO_MODEL_PATH', value='$GAZEBO_MODEL_PATH:/opt/ros/galactic/share/turtlebot3_gazebo/models')
    env_cmd3 = SetEnvironmentVariable(name='GAZEBO_MODEL_PATH', value='$GAZEBO_MODEL_PATH:/home/colcon_ws/src/aws-robomaker-hospital-world/ignition_models')


    # Launch turtlebot
    # turtlebot_dir = get_package_share_directory('nav2_bringup')
    social_navigation_dir = get_package_share_directory('social_navigation')

    world_file = os.path.join(social_navigation_dir, 'worlds', 'waffle_aws_hospital.world'),
    turtle_bot3_demo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(social_navigation_dir, 'launch/tb3_simulation_launch.py')),
        launch_arguments={
                'headless': 'False',
                # 'namespace': 'test',
                # 'use_namespace': 'True'
                'slam': 'True'
                # 'world': world_file
            }.items()
        )
    ld = LaunchDescription()
    ld.add_action(env_cmd1)
    ld.add_action(env_cmd2)
    # ld.add_action(env_cmd3)
    ld.add_action(turtle_bot3_demo)
    return ld

    