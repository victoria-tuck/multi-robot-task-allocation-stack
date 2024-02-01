FROM hardikparwana/cuda116desktop:ipopt-ros
RUN apt-get update && apt-get -y upgrade
RUN apt-get install -y ros-galactic-navigation2 ros-galactic-nav2-bringup ros-galactic-turtlebot3*
RUN apt-get install -y ros-galactic-nav2-simple-commander
RUN pip3 install setuptools
RUN pip3 install cvxpy jax jaxlib testresources cvxpylayers
RUN pip3 install --upgrade numpy
RUN pip3 install gurobipy
RUN python3 -m pip install jaxopt
RUN pip3 install -U setuptools
RUN pip3 install polytope
WORKDIR /home/
RUN echo "export PYTHONPATH=$PYTHONPATH:/home/colcon_ws/src/social_navigation/src" >> ~/.bashrc
RUN echo "source /opt/ros/galactic/setup.bash" >> ~/.bashrc
RUN echo "source /usr/share/gazebo/setup.sh" >> ~/.bashrc
RUN echo "source /home/colcon_ws/install/local_setup.bash" >> ~/.bashrc

RUN echo "alias rgazebo='ros2 launch aws_robomaker_hospital_world view_hospital.launch.py'" >> ~/.bashrc
RUN echo "alias rnav2='ros2 launch social_navigation nav2_tb3_aws_launch.py'" >> ~/.bashrc
RUN echo "alias rcsetup='ros2 launch social_navigation init_controller_setup.launch.py'" >> ~/.bashrc
RUN echo "alias rcpub='ros2 topic pub /planner_init std_msgs/msg/Bool data:\ true'" >> ~/.bashrc
RUN echo "alias rcbf='python3 /home/colcon_ws/src/social_navigation/scripts/nav2_cbf_controller.py'" >> ~/.bashrc
RUN echo "alias rcbf2='ros2 run social_navigation_py nav2_cbf --ros-args -p use_sim_time:=True'" >> ~/.bashrc
