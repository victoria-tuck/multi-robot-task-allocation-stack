# CoHAN Planner Multi

The extension of the [CoHAN Planner](https://github.com/sphanit/CoHAN_Planner) that allows the simultaneous running of multiple planners with namespace support.

# Co-operative Human Aware Navigation (CoHAN) Planner

The CoHAN Planner provides a set of packages for the Human-Aware Robot Navigation in various contexts. These packages built over ROS navigation stack, includes humans into both global and local planners to plan a human-aware trajectory for the robot considering several social criteria. Our system also provides multiple modes of planning that shift based on the context or can be set manually by simple changing the parameters.  

The system uses [Human-Aware Timed Elastic Band](https://hal.laas.fr/hal-02922029/file/Ro_Man_2020.pdf) (hateb) local planner for human-aware trajectory planning which is based on [teb_local_planner](https://github.com/rst-tu-dortmund/teb_local_planner) ROS package. 
![](https://github.com/sphanit/images/blob/main/cohan.png)


# Citation 
If you are using our code for your research, please consider citing at least one of our papers (bibtex below).

- P. T. Singamaneni, A. Favier, and R. Alami, “Human-aware navigation planner for diverse human-robot contexts,” in IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), 2021.
- P. T. Singamaneni and R. Alami, “Hateb-2: Reactive planning and decision making in human-robot co-navigation,” in International Conference on Robot & Human Interactive Communication, 2020.

# Installation (ros-melodic)
1. This installation assumes that the [ROS](http://wiki.ros.org/ROS/Installation) is already installed along with the [2D navigation stack](http://wiki.ros.org/navigation). Otherwise please install them before continuing to next steps.
2. Install the requirements
	```
	sudo apt install python-pip python-catkin-tools
	pip install scipy
	```
3. Clone the git repository
	```
	mkdir -p cohan_multi_ws/src
	cd 	cohan_multi_ws/src
	git clone https://github.com/sphanit/cohan_planner_multi.git -b main
	cd ..
	```
4. Install the dependencies using rosdep
	```
	rosdep install --from-paths src --ignore-src --rosdistro=melodic -y
	```
5. Build
	```
	catkin build
	```
	Note: catkin build might make the cpu busy utilizing all threads if not configured properly, you can instead use it with less jobs. For example, ```catkin build -j4```, can run up to four jobs at a time.

# Usage
1. It gets a bit tricky with the namespaces and remapping. CoHAN Navigation provides some examples and configuration files to launch and run it in STAGE and MORSE simulators.

2. As in the CoHAN Planner, we need to publish the known agents (or humans) on ```/tracked_agents``` topic following message structures in [cohan_msgs](https://github.com/sphanit/cohan_planner_multi/tree/master/cohan_msgs/msg).  

# Noetic Devel

Please checkout to noetic-devel for using the system in Ubuntu 20.04

# Bibtex
CoHAN
```
@inproceedings{singamaneni2021human,
  author = {Singamaneni, Phani Teja and Favier, Anthony and Alami, Rachid},
  title = {Human-Aware Navigation Planner for Diverse Human-Robot Ineraction Contexts},
  booktitle = {IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
  year = {2021},
}
```
HATEB
```
@inproceedings{singamaneni2020hateb,
  author = {Singamaneni, Phani Teja and Alami, Rachid},
  title = {HATEB-2: Reactive Planning and Decision making in Human-Robot Co-navigation},
  booktitle = {International Conference on Robot \& Human Interactive Communication},
  year = {2020},
  doi={10.1109/RO-MAN47096.2020.9223463}}
}
```
