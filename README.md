## Hardware requirements
- NVIDIA discrete GPU (for gazbeo)
- Diplay should use NVIDA and not integrated graphics
- Docker installed (docker and nvidia-docker)
- https://docs.docker.com/engine/install/ubuntu/
- https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html



This documentation is for `ozgur` branch of `hsr` repository and of this repository. The video accompanying this documentation can be found at 
https://drive.google.com/file/d/144wxIIqVRloF_41ngfn34iR-X_7aCdBs/view?usp=drive_link

To setup SMrTa, first compile bitwuzla by running
```
git submodule init
git submodule update
cd colcon_ws/src/social_navigation/social_navigation_py/social_navigation_py/SMrTa
sh setup_bitwuzla.sh
```

To prepare the environment, first, mount the correct folder in docker-compose.yaml. The colcon_ws (ROS2) workspace should be in mounted at /home/colcon_ws. Then to build the environment, run
```
docker compose build
docker compose up -d
docker exec -it hsr-ros-1 bash
```
Next, you need to make and install the social force model library
```
cd /home/colcon_ws/src/lightsfm
make
make install
```

Now, you have to first build the colcon (ROS2) workspace. Navigate to
```
cd /home/colcon_ws
colcon build --symlink-install
```
Note sometimes ROS does not figure out package dependency order properly when multiple ROS packages are present. In this case, it may take multiple runs of colon build to be successful. 

Now source the installed packages with following command
```
source install/local_setup.bash
```
Finally, to give docker environment permission to use graphics of hist machine, run the following command **from host machine**
```
xhost +
```

Change number of humans and obstacle in following files
```
nav2_cbf_controller.py: 
- line for self.num_humans, and self.num_obstacles
get_human_states.cpp : 
- line this->declare_parameter<int>("num_humans",25)
```
Choose nominal controller gains and CBF parameters in 
```
cbf_obstacle_controller.py
```

Then run the codes in the following sequence. To aid in implementation, several aliases are defined in `~/.bashrc` file.

1. To launch the gazebo environment with the robot inside it

```
rgazebo input_file:=<setup_file>
```

2. To launch the ROS2 navigation stack (to use its planners)
```
rnav2
```

3. To get human states from gazebo and to find closest obstacle points to robot
```
rcsetup input_file:=<setup_file>

```

4. To start social force model
```
rsfm
```

5. To start the user controller for one robot:
```
rcbf2r1
```
OR
for many robots:
```
rcbf <number_of_robots>
```
When using the second option, nav2_cbf processes will need to be killed using `ps` to find the pid and then `kill <pid>` for each process.

6. When the controller status is ONLINE, run the following command to set goal and run controller simulation
```
rcpub
```

7. To manage the task assignment for each agent:
```
rcset input_file:=<setup_file>
```

8. For task allocation and high-level path planning for a set of agents:
```
rcdis -p input_file:=<setup_file>
```

Now you can use `rviz` to give the robot a goal. You can use `Nav2 Goal` button to send let ROS2 Navigation2 package plan the path and also control the robot, or you can publish to `2D Goal` button to send the goal location to the nav2_cbf_controller node that we ran in the last line. (Note: u need to add the 2D Goal button on rviz. See the attached video on how to do that).

Note: ROS2 Navigation Stack requires that you set the initial pose first so that it can start robot localization. This is done in the video too before sending goal waypoint.
