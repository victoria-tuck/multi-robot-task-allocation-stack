import os
import sys

import launch
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # world_file_name = "hospital.world"
    # world = os.path.join(get_package_share_directory('aws_robomaker_hospital_world'), 'worlds', world_file_name)

    # world_file_name = "waffle_aws_hospital.world"
    world_file_name = "waffle_aws_hospital_humans.world"
    # world_file_name = "test_world.world"
    # world_file_name = "test_human.world"
    # world_file_name = "hospital.world"
    world = os.path.join(get_package_share_directory('social_navigation'), 'worlds', world_file_name)
    
    gazebo_params_path = os.path.join(
                  get_package_share_directory('social_navigation'),'configs','gazebo_params.yaml')
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
                    launch_arguments={'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_path, 
                                      'world': world,
                                      'gui': 'true'}.items(),

            )

    # gazebo_ros = get_package_share_directory('gazebo_ros')
    # gazebo_client = launch.actions.IncludeLaunchDescription(
	# launch.launch_description_sources.PythonLaunchDescriptionSource(
    #         os.path.join(gazebo_ros, 'launch', 'gzclient.launch.py')),
    #     condition=launch.conditions.IfCondition(launch.substitutions.LaunchConfiguration('gui'))
    #     # launch_arguments={'extra_gazebo_args': '--ros-args -p -publish_rate:=100 '}.items()
    #     # launch_arguments={'extra_gazebo_args': '--ros-args --params-file ' + '/home/colcon_ws/src/social_navigation/configs/gazebo_params.yaml'}.items()
    #  )
    # gazebo_server = launch.actions.IncludeLaunchDescription(
    #     launch.launch_description_sources.PythonLaunchDescriptionSource(
    #         os.path.join(gazebo_ros, 'launch', 'gzserver.launch.py')),
    #         launch_arguments={'extra_gazebo_args': '--ros-args -p -publish_rate:=100 '}.items()
    # )


    ld = launch.LaunchDescription([
        gazebo
        # launch.actions.DeclareLaunchArgument(
        #   'world',
        #   default_value=[world, ''],
        #   description='SDF world file'),
        # launch.actions.DeclareLaunchArgument(
        #     name='gui',
        #     default_value='false'
        # ),
        # gazebo_server,
        # gazebo_client
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
