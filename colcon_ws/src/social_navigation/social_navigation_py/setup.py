from setuptools import setup

package_name = 'social_navigation_py'
submodules = "social_navigation_py/utils"

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, submodules],
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
            'nav2_mppi = social_navigation_py.nav2_mppi_controller:main',
        ],
    },
)
