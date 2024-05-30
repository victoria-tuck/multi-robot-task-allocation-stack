import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PythonExpression

def generate_launch_description():
    
    # Set environment variable
    env_cmd1 = SetEnvironmentVariable(name='TURTLEBOT3_MODEL', value='waffle')
    env_cmd4 = SetEnvironmentVariable(name='TURTLEBOT3_MODEL', value='waffle2')
    env_cmd2 = SetEnvironmentVariable(name='GAZEBO_MODEL_PATH', value='$GAZEBO_MODEL_PATH:/opt/ros/galactic/share/turtlebot3_gazebo/models')
    env_cmd3 = SetEnvironmentVariable(name='GAZEBO_MODEL_PATH', value='$GAZEBO_MODEL_PATH:/home/colcon_ws/src/aws-robomaker-hospital-world/ignition_models')


    # Launch turtlebot
    # turtlebot_dir = get_package_share_directory('nav2_bringup')
    social_navigation_dir1 = get_package_share_directory('social_navigation')
    social_navigation_dir2 = get_package_share_directory('social_navigation')

    # world_file = os.path.join(social_navigation_dir, 'worlds', 'waffle_aws_hospital.world'),
    turtle_bot3_1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(social_navigation_dir1, 'launch/tb3_simulation_launch.py')),
        launch_arguments={
                'headless': 'False',
                'use_simulator': 'False',
                'namespace': 'tb3',
                'use_namespace': 'True',
                'slam': 'False',
                'map': os.path.join(social_navigation_dir1, 'worlds', 'map_aws', 'my_map.yaml')
                # 'world': world_file
            }.items()
        )
    turtle_bot3_2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(social_navigation_dir2, 'launch/tb3_simulation_launch.py')),
        launch_arguments={
                'headless': 'False',
                'use_simulator': 'False',
                'namespace': 'tb3_adv',
                'use_namespace': 'True',
                'slam': 'False',
                'map': os.path.join(social_navigation_dir2, 'worlds', 'map_aws', 'my_map.yaml')
                # 'world': world_file
            }.items()
        )
    
    
    ld = LaunchDescription([
        # PushRosNamespace('tb3'),
        # turtle_bot3_1, 
        # PushRosNamespace('tb3_adv'),
        # turtle_bot3_2,
    ])
    ld.add_action(env_cmd1)
    ld.add_action(env_cmd2)
    ld.add_action(env_cmd4)
    PushRosNamespace('tb3')
    ld.add_action(turtle_bot3_1)
    PushRosNamespace('tb3_adv')
    ld.add_action(turtle_bot3_2)
    # ld.add_action(env_cmd3)
    # ld.add_action(turtle_bot3_1)
    # ld.add_action(turtle_bot3_2)
    return ld

# FOR SLAM and making a map
#ros2 launch aws_robomaker_hospital_world view_hospital.launch.py
#ros2 launch social_navigation nav2_tb3_aws_launch.py
#ros2 run turtlebot3_teleop teleop_keyboard
#ros2 run nav2_map_server map_saver_cli -f map6