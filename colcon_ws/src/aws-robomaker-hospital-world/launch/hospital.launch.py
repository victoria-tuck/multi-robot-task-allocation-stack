import os
import sys

import launch
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
   
    # world_file_name = "waffle_aws_hospital.world"
    world_file_name = "aws_hospital_humans.world"
    world_file_name = "aws_hospital_humans_new_plugin.world"
    # world_file_name = "hospital.world"
    world = os.path.join(get_package_share_directory('social_navigation'), 'worlds', world_file_name)

    aws_launch_dir = os.path.join( get_package_share_directory('aws_robomaker_hospital_world'), 'launch')
    social_navigation_launch_dir = os.path.join( get_package_share_directory('social_navigation'), 'launch')
    social_navigation_dir = get_package_share_directory('social_navigation')
    use_simulator = LaunchConfiguration('use_simulator')
    namespace = LaunchConfiguration('namespace')

    headless = LaunchConfiguration('headless')

    # world = LaunchConfiguration('world')
    pose = {'x': LaunchConfiguration('x_pose', default='-4.9'),
            'y': LaunchConfiguration('y_pose', default='13.8'),
            'z': LaunchConfiguration('z_pose', default='0.01'),
            'R': LaunchConfiguration('roll', default='0.00'),
            'P': LaunchConfiguration('pitch', default='0.00'),
            'Y': LaunchConfiguration('yaw', default='0.00')}
    pose2 = {'x': LaunchConfiguration('x_pose', default='-3.0'),
            'y': LaunchConfiguration('y_pose', default='11.0'),
            'z': LaunchConfiguration('z_pose', default='0.01'),
            'R': LaunchConfiguration('roll', default='0.00'),
            'P': LaunchConfiguration('pitch', default='0.00'),
            'Y': LaunchConfiguration('yaw', default='0.00')}
    robot_name = LaunchConfiguration('robot_name')
    robot_sdf = LaunchConfiguration('robot_sdf')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')
    
    declare_robot_name_cmd = DeclareLaunchArgument(
        'robot_name',
        default_value='turtlebot3_waffle',
        description='name of the robot')

    declare_robot_sdf_cmd = DeclareLaunchArgument(
        'robot_sdf',
        default_value=os.path.join(social_navigation_dir, 'worlds', 'waffle.model'),
        description='Full path to robot sdf file to spawn the robot in gazebo')
    
    declare_use_simulator_cmd = DeclareLaunchArgument(
        'use_simulator',
        default_value='True',
        description='Whether to start the simulator')
    
    gazebo_params_path = os.path.join(
                  get_package_share_directory('social_navigation'),'configs','gazebo_params.yaml')
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
                    launch_arguments={'extra_gazebo_args': '--verbose --ros-args --params-file ' + gazebo_params_path, 
                                      'world': world,
                                      'gui': 'true'}.items(),

            )
    
    start_gazebo_spawner_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=[
            '-entity', robot_name,
            '-file', robot_sdf,
            '-robot_namespace', namespace,
            '-x', pose['x'], '-y', pose['y'], '-z', pose['z'],
            '-R', pose['R'], '-P', pose['P'], '-Y', pose['Y']])
    
    start_gazebo_spawner_cmd2 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=[
            '-entity', 'adv',
            '-file', robot_sdf,
            '-robot_namespace', '/adv',
            '-x', pose2['x'], '-y', pose2['y'], '-z', pose2['z'],
            '-R', pose2['R'], '-P', pose2['P'], '-Y', pose2['Y']])



    ld = launch.LaunchDescription()
    
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_robot_name_cmd)
    ld.add_action(declare_robot_sdf_cmd)
    ld.add_action(gazebo)
    ld.add_action(start_gazebo_spawner_cmd)
    # ld.add_action(start_gazebo_spawner_cmd2)
    return ld


if __name__ == '__main__':
    generate_launch_description()
