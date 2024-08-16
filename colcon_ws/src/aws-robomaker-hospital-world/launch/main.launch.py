from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os

def launch_nav_launch(context, *args, **kwargs):
    social_nav_dir = get_package_share_directory('social_navigation')
    nav_launch = os.path.join(social_nav_dir, 'launch/nav2_tb3_aws_launch.py')
    setup_launch = os.path.join(social_nav_dir, 'launch/init_controller_setup.launch.py')
    map = LaunchConfiguration('map')
    declare_map_arg = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(social_nav_dir, 'worlds', 'map_aws', 'my_map.yaml'),
        description='Map yaml file')
    
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav_launch),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(setup_launch)
        )
    ]

def launch_gazebo_launch(context):
    world_dir = get_package_share_directory('aws_robomaker_hospital_world')
    gazebo_launch = os.path.join(world_dir, 'launch/view_hospital.launch.py')
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch)
    )]

def launch_cbf_launch(context):
    social_nav_dir = get_package_share_directory('social_navigation')
    cbf_launch = os.path.join(social_nav_dir, 'launch/multi_cbf.launch.py')
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(cbf_launch),
        launch_arguments={
            'use_sim_time': 'true'
        }.items()
    )]

def launch_goal_setter_launch(context):
    social_nav_dir = get_package_share_directory('social_navigation')
    goal_setter_launch = os.path.join(social_nav_dir, 'launch/goal_setter.launch.py')
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(goal_setter_launch),
        launch_arguments={
            'use_sim_time': 'true'
        }.items()
    )]


def generate_launch_description():

    world_dir = get_package_share_directory('aws_robomaker_hospital_world')
    social_nav_dir = get_package_share_directory('social_navigation')

    gazebo_launch = os.path.join(world_dir, 'launch/view_hospital.launch.py')
    nav_launch = os.path.join(social_nav_dir, 'launch/nav2_tb3_aws_launch.py')
    setup_launch = os.path.join(social_nav_dir, 'launch/init_controller_setup.launch.py')

    # map = LaunchConfiguration('map')
    # declare_map_arg = DeclareLaunchArgument(
    #     'map',
    #     default_value=os.path.join(social_nav_dir, 'worlds', 'map_aws', 'my_map.yaml'),
    #     description='Map yaml file')

    planner_initializer = Node(
        package='social_navigation_py',
        executable='init_planner',
        output='screen',
        parameters=[{
            'use_sim_time': True}]
    )

    ld = LaunchDescription()
    # ld.add_action(TimerAction(
    #     period=20.0,
    #     actions=[OpaqueFunction(function=launch_gazebo_launch)]
    # ))
    # ld.add_action(TimerAction(
    #                 period=30.0,
    #                 actions=[]))
    # ld.add_action(declare_map_arg)
    # ld.add_action(TimerAction(
    #     period=0.0,
    #     actions=[OpaqueFunction(function=launch_nav_launch)])
    # )

    ld.add_action(TimerAction(
        period=0.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(setup_launch),
            launch_arguments={
                'use_sim_time': 'true'
            }.items()
        )]
    ))
    ld.add_action(TimerAction(
        period=15.0,
        actions=[OpaqueFunction(function=launch_cbf_launch)]
    ))

    ld.add_action(TimerAction(
        period=20.0,
        actions=[planner_initializer]
    ))

    ld.add_action(TimerAction(
        period=25.0,
        actions=[OpaqueFunction(function=launch_goal_setter_launch)]
    ))
    

    return ld

# IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(nav_launch),
#         launch_arguments={
#                             'headless': 'false',
#                             'use_simulator': 'false',
#                             # 'namespace': 'robot1',
#                             # 'use_namespace': 'True',
#                             'slam': 'false',
#                             'map': map,
#                             'use_sim_time': 'true'
#                             # 'world': world_file
#                         }.items()
#     )
    # return LaunchDescription([
    #     IncludeLaunchDescription(
    #         PythonLaunchDescriptionSource(nav_launch)
    #     )
    # ])
    # return LaunchDescription([
    #     IncludeLaunchDescription(
    #         PythonLaunchDescriptionSource(gazebo_launch)
    #     ),
    #     IncludeLaunchDescription(
    #         PythonLaunchDescriptionSource(nav_launch)
    #     )
    # ])
    # return LaunchDescription([
    #     TimerAction(
    #         period=0.0,
    #         actions=[
    #             IncludeLaunchDescription(
    #                 PythonLaunchDescriptionSource(gazebo_launch)
    #             ),
    #             TimerAction(
    #                 period=30.0,
    #                 actions=[
    #                     IncludeLaunchDescription(
    #                         PythonLaunchDescriptionSource(nav_launch)
    #                     )
    #                 ]
    #             )
    #         ]
    #     )
    # ])

    return LaunchDescription([
        declare_map_arg,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch),
            launch_arguments={
                'headless': 'False',
                'use_simulator': 'False',
                # 'namespace': 'robot1',
                # 'use_namespace': 'True',
                'slam': 'False',
                'map': map,
                'use_sim_time': 'True'
                # 'world': world_file
            }.items()
        ),
        GroupAction(
            actions=[
                TimerAction(
                    period=10.0,
                    actions=[IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(nav_launch),
                        launch_arguments={
                            'headless': 'false',
                            'use_simulator': 'false',
                            # 'namespace': 'robot1',
                            # 'use_namespace': 'True',
                            'slam': 'false',
                            'map': map,
                            'use_sim_time': 'true'
                            # 'world': world_file
                        }.items()
                        )
                    ]
                )
            ]
        )
    ])


def parse_arguments(argv):
    args = []
    for arg in argv:
        if ":=" in arg:
            parsed_arg = arg.split(':')
            print(parsed_arg)
            args.append(f"-{parsed_arg[0]}")
            args.append(parsed_arg[1][1:])
    return args
