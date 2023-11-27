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
    # world_file_name = "waffle_aws_hospital_humans.world"
    world_file_name = "two_waffle_aws_hospital_humans.world"
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

    ld = launch.LaunchDescription([
        gazebo
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
