from setuptools import setup

package_name = 'social_navigation_py'
submodules = "social_navigation_py/utils"
sfm_submodules = "social_navigation_py/socialforce"

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, submodules, sfm_submodules],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='hardiksp@umich.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'nav2_cbf = social_navigation_py.nav2_cbf_controller:main',
            'output = social_navigation_py.terminal_output:main',
            'nav2_mppi = social_navigation_py.nav2_mppi_controller:main',
            'human_sfm = social_navigation_py.human_control_node:main',
            'human_stop = social_navigation_py.human_stop_node:main',
            'minimal_publisher = social_navigation_py.minimal_publisher:main',
            'goal_setter = social_navigation_py.goal_setter:main',
            'goal_setter_for_travel_time = social_navigation_py.travel_time_collector:main',
            'dispatcher = social_navigation_py.dispatcher:main',
            'init_planner = social_navigation_py.init_planner:main', 
            'image_saver = social_navigation_py.image_saver:main',
            'planner_wrapper = social_navigation_py.nav2_planner_wrapper:main',
            'room_queue = social_navigation_py.room_queue:main',
            'terminal_output = social_navigation_py.terminal_output:main'
        ],
    },
)
