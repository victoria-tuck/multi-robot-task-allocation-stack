import os
import sys

import launch
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace

def generate_launch_description():
   
    # world_file_name = "waffle_aws_hospital.world"
    world_file_name = "aws_hospital_humans.world"
    world_file_name = "aws_hospital_humans_new_plugin3.world"
    world_file_name = "aws_hospital_no_humans.world"
    # world_file_name = "hospital.world"
    world = os.path.join(get_package_share_directory('social_navigation'), 'worlds', world_file_name)

    aws_launch_dir = os.path.join( get_package_share_directory('aws_robomaker_hospital_world'), 'launch')
    social_navigation_launch_dir = os.path.join( get_package_share_directory('social_navigation'), 'launch')
    social_navigation_dir = get_package_share_directory('social_navigation')
    use_simulator = LaunchConfiguration('use_simulator')
    namespace = LaunchConfiguration('namespace')

    headless = LaunchConfiguration('headless')

    # world = LaunchConfiguration('world')
    base_pose = {'x': LaunchConfiguration('x_pose', default='1.2'),
            'y': LaunchConfiguration('y_pose', default='15.6'),
            'z': LaunchConfiguration('z_pose', default='0.01'),
            'R': LaunchConfiguration('roll', default='0.00'),
            'P': LaunchConfiguration('pitch', default='0.00'),
            'Y': LaunchConfiguration('yaw', default='0.00')}
    # pose1 = {'x': LaunchConfiguration('x_pose', default='-4.9'),
    #         'y': LaunchConfiguration('y_pose', default='13.8'),
    #         'z': LaunchConfiguration('z_pose', default='0.01'),
    #         'R': LaunchConfiguration('roll', default='0.00'),
    #         'P': LaunchConfiguration('pitch', default='0.00'),
    #         'Y': LaunchConfiguration('yaw', default='0.00')}
    pose1 = {'x': LaunchConfiguration('x_pose', default='0.00'),
            'y': LaunchConfiguration('y_pose', default='2.20'),
            'z': LaunchConfiguration('z_pose', default='0.01'),
            'R': LaunchConfiguration('roll', default='0.00'),
            'P': LaunchConfiguration('pitch', default='0.00'),
            'Y': LaunchConfiguration('yaw', default='0.00')}
    pose2 = {'x': LaunchConfiguration('x_pose', default='4.25'),
            'y': LaunchConfiguration('y_pose', default='-27.5'),
            'z': LaunchConfiguration('z_pose', default='0.01'),
            'R': LaunchConfiguration('roll', default='0.00'),
            'P': LaunchConfiguration('pitch', default='0.00'),
            'Y': LaunchConfiguration('yaw', default='0.00')}
    robot_name = LaunchConfiguration('robot_name')
    robot_sdf = LaunchConfiguration('robot_sdf')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        # default_value='robot1',
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
                  get_package_share_directory('social_navigation'),'configs','gazebo_params.yml')
    print(f"***************** params file: {gazebo_params_path}")
    # gazebo = IncludeLaunchDescription(
    #             PythonLaunchDescriptionSource([os.path.join(
    #                 get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
    #                 launch_arguments={'extra_gazebo_args': '--verbose --ros-args --params_file ' + gazebo_params_path, 
    #                                   'world': world,
    #                                   'gui': 'true'}.items(),

    #         )
    print(f"world: {world}") #/home/colcon_ws/install/social_navigation/share/social_navigation/worlds/aws_hospital_humans_new_plugin.world

    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
                    launch_arguments={'params_file': gazebo_params_path, 
                                      'world': world,
                                      'gui': 'true'}.items(),
            )
    #ros2 launch gazebo_ros gazebo.launch.py params_file:=params.yml
    start_gazebo_spawner_cmd_base = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=[
            '-entity', robot_name,
            '-file', robot_sdf,
            # '-robot_namespace', 'robot1',
            '-robot_namespace', namespace,
            '-x', base_pose['x'], '-y', base_pose['y'], '-z', base_pose['z'],
            '-R', base_pose['R'], '-P', base_pose['P'], '-Y', base_pose['Y']])
    
    start_gazebo_spawner_cmd1 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=[
            '-entity', 'robot1',
            '-file', robot_sdf,
            '-robot_namespace', 'robot1',
            # '-robot_namespace', namespace,
            '-x', pose1['x'], '-y', pose1['y'], '-z', pose1['z'],
            '-R', pose1['R'], '-P', pose1['P'], '-Y', pose1['Y']])

    start_gazebo_spawner_cmd2 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=[
            '-entity', 'robot2',
            '-file', robot_sdf,
            '-robot_namespace', 'robot2',
            # '-robot_namespace', namespace,
            '-x', pose2['x'], '-y', pose2['y'], '-z', pose2['z'],
            '-R', pose2['R'], '-P', pose2['P'], '-Y', pose2['Y']])
    
    # start_gazebo_human_spawner_cmd = Node(
    #     package='gazebo_ros',
    #     executable='spawn_entity.py',
    #     output='screen',
    #     arguments=[
    #         '-entity', 'actor3',
    #         '-file', os.path.join(social_navigation_dir, 'worlds', 'human_actor.model'),
    #         '-robot_namespace', '/actor3',
    #         '-x', '1.0', '-y', '6.0', '-z', '1.0',
    #         '-R', '0.0', '-P', '0.0', '-Y', '3.14'])



    ld = launch.LaunchDescription()
    
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_robot_name_cmd)
    ld.add_action(declare_robot_sdf_cmd)
    ld.add_action(gazebo)
    ld.add_action(start_gazebo_spawner_cmd_base)
    PushRosNamespace('robot1')
    ld.add_action(start_gazebo_spawner_cmd1)
    # ld.add_action(start_gazebo_human_spawner_cmd)
    PushRosNamespace('robot2')
    ld.add_action(start_gazebo_spawner_cmd2)
    return ld


if __name__ == '__main__':
    generate_launch_description()
