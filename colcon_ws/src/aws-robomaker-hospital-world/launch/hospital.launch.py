import os
import sys

import launch
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import GroupAction

import argparse
import time
import json

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

    arguments = parse_arguments(sys.argv)

    scenario_file = get_scenario_file_from_arguments(arguments)
    positions = get_robot_positions(scenario_file)

    robot_names = [f'robot{i + 1}' for i in range(len(positions))]  
    poses = [{'x': LaunchConfiguration('x_pose', default=str(position[0])),
            'y': LaunchConfiguration('y_pose', default=str(position[1])),
            'z': LaunchConfiguration('z_pose', default='0.01'),
            'R': LaunchConfiguration('roll', default='0.00'),
            'P': LaunchConfiguration('pitch', default='0.00'),
            'Y': LaunchConfiguration('yaw', default='0.00')}
            for position in positions]
    print(poses)
    
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

    launch_gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
                    launch_arguments={'params_file': gazebo_params_path, 
                                      'world': world,
                                      'gui': 'true'}.items(),
            )
    
    names = robot_names
    namespaces = robot_names
    robot_spawners = [ Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=[
            '-entity', name,
            '-file', robot_sdf,
            '-robot_namespace', robot_namespace,
            '-x', pose['x'], '-y', pose['y'], '-z', pose['z'],
            '-R', pose['R'], '-P', pose['P'], '-Y', pose['Y']]) 
            for name, robot_namespace, pose in zip(names, namespaces, poses)]
    
    static_map_odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_odom_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )
    
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
    ld.add_action(launch_gazebo)
    ld.add_action(static_map_odom_tf)
    base_init = False
    for spawner, namespace in zip(robot_spawners, namespaces):
        namespaced_group = GroupAction([
            PushRosNamespace(namespace),
            spawner
        ])
        ld.add_action(namespaced_group)

    for relay in create_relays():
        ld.add_action(relay)

    return ld

def create_relays():
    """Creates relays for robot1 topics"""
    
    essential_topics = [
        'scan',
        'odom', 
        'tf',
        'joint_states',
        'imu' # Raw color only
    ]
    
    relays = []
    
    for topic in essential_topics:
        relay_name = f'robot1_{topic.replace("/", "_")}_to_global'
        relays.append(
            Node(
                package='topic_tools',
                executable='relay',
                name=relay_name,
                arguments=[f'robot1/{topic}', topic],
                output='screen'
            )
        )
    
    return relays


def get_robot_positions(file):
    with open(file, 'r') as f:
        scenario_setup = json.load(f) 
    positions = [] 
    for robot in scenario_setup["agents"].values():
        print(f"Robot: {robot}")
        positions.append(robot["start"])
    return positions

def get_scenario_file_from_arguments(argv):
    print(f"Arguments: {argv}")
    parser = argparse.ArgumentParser(
        description='Start robot goal setters'
    )
    parser.add_argument('-input_file', type=str, help='Scenario file')
    args = parser.parse_args(argv)
    return args.input_file

def parse_arguments(argv):
    args = []
    for arg in argv:
        if ":=" in arg:
            parsed_arg = arg.split(':')
            print(parsed_arg)
            if not parsed_arg[0] == 'use_sim_time' and not parsed_arg[0] == 'autostart':
                args.append(f"-{parsed_arg[0]}")
                args.append(parsed_arg[1][1:])
    return args


if __name__ == '__main__':
    generate_launch_description()