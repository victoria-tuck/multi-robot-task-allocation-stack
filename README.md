This documentation is for `ros2` branch of `cbfkit-internal` repository and of this repository. The video accompanying this documentation can be found at 
https://myteams.toyota.com/:v:/s/TRINA-FRD-CPS-HARDIK/EZ5ej32o_klDppt9FSFJDUYBn3FaR1NsMa5L5w3jIXO-jw?nav=eyJyZWZlcnJhbEluZm8iOnsicmVmZXJyYWxBcHAiOiJTdHJlYW1XZWJBcHAiLCJyZWZlcnJhbFZpZXciOiJTaGFyZURpYWxvZyIsInJlZmVycmFsQXBwUGxhdGZvcm0iOiJXZWIiLCJyZWZlcnJhbE1vZGUiOiJ2aWV3In19&e=PM978o

To prepare the environment, first, mount the correct folder in docker-compose.yaml. The colcon_ws (ROS2) workspace should be in mounted at /home/colcon_ws. Then to build the environment, run
```
docker compose build
docker compose up -d
docker exec -it hsr-ros-1 bash
```

Now, you have to first build the colcon (ROS2) workspace. Navigate to
```
cd /home/colcon_ws
colcon build --symlink-install
```
Note sometimes ROS does not figure out package dependency order properly when multiple ROS packages are present. In this case, it may take multiple runs of colon build to be successful. Next, you need to make and install the social force model library
```
cd /home/colcon_ws/src/lightsfm
make
make install
```

After this, from each terminal, you need to source the following two lines
```
source /opt/ros/galactic/setup.bash
source /home/colcon_ws/install/local_setup.bash
```

Then run the codes in the following sequence

1. To launch the gazebo environment with the robot inside it

```
ros2 launch aws_robomaker_hospital_world view_hospital.launch.py 
```

2. To launch the ROS2 navigation stack (to use its planners)
```
ros2 launch social_navigation nav2_tb3_aws_launch.py
```

3. To get human states from gazebo and publish it
```
ros2 run social_navigation get_human_states 

```

4 Finally, to run the user controller
```
python3 colcon_ws/src/social_navigation/scripts/nav2_cbf_controller.py
```

Now you can use `rviz` to give the robot a goal. You can use `Nav2 Goal` button to send let ROS2 Navigation2 package plan the path and also control the robot, or you can publish to `2D Goal` button to send the goal location to the nav2_cbf_controller node that we ran in the last line. (Note: u need to add the 2D Goal button on rviz. See the attached video on how to do that).

Note: ROS2 Navigation Stack requires that you set the initial pose first so that it can start robot localization. This is done in the video too before sending goal waypoint.
