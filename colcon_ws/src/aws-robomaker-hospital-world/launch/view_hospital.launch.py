import os
import sys

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch.actions import SetEnvironmentVariable


def generate_launch_description():
    ld = launch.LaunchDescription([
        SetEnvironmentVariable(name='GAZEBO_MODEL_PATH', value='$GAZEBO_MODEL_PATH:/home/colcon_ws/src/aws-robomaker-hospital-world/ignition_models'),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                [get_package_share_directory(
                    'aws_robomaker_hospital_world'), '/launch/hospital.launch.py']
            ),
            launch_arguments={
                'gui': 'true'
            }.items()
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
