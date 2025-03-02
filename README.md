## Hardware requirements
- NVIDIA discrete GPU (for gazbeo)
- Diplay should use NVIDA and not integrated graphics
- Docker installed (docker and nvidia-docker)
- https://docs.docker.com/engine/install/ubuntu/
- https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html

## Installation
This documentation is for `main` branch of `multi-robot-task-allocation-stack` repository and of this repository. The video accompanying this documentation can be found at <>

To setup the SMrTa task allocator, first add the submodules running
```
git submodule init
git submodule update
cd colcon_ws/src/social_navigation/social_navigation_py/social_navigation_py/SMrTa
git submodule init
git submodule update
```

Then install the [bitwuzla](https://github.com/bitwuzla/bitwuzla/blob/main/docs/install.rst) Python bindings dependencies. Bitwuzla can be installed by running

```
sh setup_bitwuzla.sh
```
from the SMrTa folder.

Then to build the environment, run
```
docker compose build
docker compose up -d
docker exec -it multi-robot-task-allocation-stack-ros-1 bash
```
from the highest level of the repository.

Now, you have to first build the colcon (ROS2) workspace. Navigate to
```
cd /home/colcon_ws
colcon build --symlink-install
```
Note sometimes ROS does not figure out package dependency order properly when multiple ROS packages are present. In this case, it may take multiple runs of colcon build to be successful. If one error is shown, after this step, you can still proceed successfully.

Now source the installed packages with following command
```
source install/local_setup.bash
```
Finally, to give docker environment permission to use graphics of hist machine, run the following command **from host machine**
```
xhost +
```

## Customization
Choose nominal controller gains and CBF parameters in `cbf_obstacle_controller.py`.

To change the number of controllers that are started, change the case config file in `multi_cbf.launch.py` to your config file or change the name of the file to `case_config.yaml`

## Running the Code
Then run the code in the following sequence. To aid in implementation, several aliases are defined in the `~/.bashrc` file upon docker build. Six terminals will be needed; run the docker exec command in each terminal. Wait for each of the below commands to complete before running the next.

1. To launch the gazebo environment with the robot inside it

```
rgazebo input_file:=<path_to_setup_file>
```
Example:
```
rgazebo input_file:=/home/colcon_ws/src/social_navigation/social_navigation_py/social_navigation_py/robot_setup_6.json
```
Note: After this step, the Gazebo environment should the robots (which are in the middle of blue circles). Upon start-up, the robot installation will occasionally fail. If this occurs, exit and rerun the above command.

2. To launch the ROS2 navigation stack (to use its planners)
```
rnav2
```

3. To launch multiple robotic agent navigation stacks
```
ros2 launch aws_robomaker_hospital_world main.launch.py input_file:=<path_to_setup_file>
```
Example:
```
ros2 launch aws_robomaker_hospital_world main.launch.py input_file:=/home/colcon_ws/src/social_navigation/social_navigation_py/social_navigation_py/robot_setup_6.json
```
Note: After this step, the rviz environment should include an arrow for each robot and colored buffers around the walls. If it does not, rerun this command.

4. To launch the navigation stack wrapper
```
rcplan --ros-args -p "robots:=<list_of_robot_names>"
```
Example:
```
rcplan --ros-args -p "robots:=["robot1", "robot2", "robot3", "robot4", "robot5", "robot6"]"
```

5. To launch the room queues
```
rqueues
```

6. **This will start moving the robots.** For task allocation and high-level path planning for a set of agents:
```
rcdis -p input_file:=<setup_file>
```
Example:
```
rcdis -p input_file:=/home/colcon_ws/src/social_navigation/social_navigation_py/social_navigation_py/robot_setup_6.json
```

## Experiments

To run the experiment included in our paper, run the examples from the 6 steps above. Calculation time data can be processed afterwards with

```
python3 process_data.py
```

which will print out the results of Table 1.

## Debugging
Many of the nodes have been combined into single launch files for ease of use. However, additional aliases have been included to run these nodes separately when debugging.

1. To get human states from gazebo and to find the closest obstacle points to each robot
```
rcsetup input_file:=<setup_file>

```

2. To start the user controller for multiple robots:
```
multi_rcbf
```
with the configuration specified in ```src/social_navigation/social_navigation/configs/case_config.yaml```

3. When the controller status is ONLINE, run the following command to set goal and run controller simulation
```
rcpub
```

4. To manage the task assignment for each agent:
```
rcset input_file:=<setup_file>
```

## Data Collection
The TravelTimeCollector node in travel_time_collector.py has been provided in order to collect travel time data. This can be run with:
```
rctimecollect -p time_collection_params:=<params_file> -p save_file:=<save_file>
```
where <params_file> is the name of a json file containing the number of iterations, save mode, desired format, map locations, and location ids and <save_file> is where the travel time information should be saved.

## Scenic Integration

To integrate with Scenic, navigate to the src folder then install with the following commands:
```
git clone git:github.com:Kai-X-Org/ScenicROS2.git
cd ScenicROS2
python3 -m pip install -e .
```

The bookshelf example can be run with
```
cd src/scenic/simulators/Gazebo
scenic test.scenic --simulate
```

Further documentation can be found on the [documentation page](https://docs.scenic-lang.org/en/latest/). 
