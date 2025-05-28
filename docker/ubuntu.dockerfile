# Need this cuda image
FROM hardikparwana/cuda118desktop:ros-humble-rmf

RUN apt-get update
RUN apt-get install -y wget build-essential libncursesw5-dev libssl-dev libsqlite3-dev tk-dev libgdbm-dev libc6-dev libbz2-dev libffi-dev zlib1g-dev
RUN apt-get install -y software-properties-common
RUN apt-get install -y curl
RUN curl -sS https://bootstrap.pypa.io/get-pip.py | python3.10
RUN python3 -m pip install numpy
RUN apt-get install -y python3.10-dev

RUN apt-get install -y ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-turtlebot3*
RUN apt-get install -y ros-humble-nav2-simple-commander
RUN pip3 install -U setuptools
RUN pip3 install polytope numpy cvxpy jax jaxlib testresources cvxpylayers gurobipy

RUN python3 -m pip install setuptools==58.2.0
RUN python3 -m pip install numpy==1.26.4 matplotlib
RUN python3 -m pip install --upgrade "jax[cuda11_pip]==0.4.25" -f https://storage.googleapis.com/jax-releases/jax_cuda_releases.html jaxlib==0.4.25
RUN python3 -m pip install matplotlib==3.7.1 pillow==9.5.0 kiwisolver==1.4.4 polytope
RUN python3 -m pip install myst-parser sphinx sphinx-rtd-theme

RUN echo "export PYTHONPATH=\$PYTHONPATH:/home/colcon_ws/src/social_navigation/src" >> ~/.bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source /usr/share/gazebo/setup.sh" >> ~/.bashrc
RUN echo "source /home/colcon_ws/install/local_setup.bash" >> ~/.bashrc

WORKDIR /home/

RUN git clone https://github.com/robotics-upo/lightsfm.git
WORKDIR /home/lightsfm
RUN make && make install

# Setup SMrTa
ADD colcon_ws/src/social_navigation/social_navigation_py/social_navigation_py/SMrTa /home/colcon_ws/src/social_navigation/social_navigation_py/social_navigation_py/SMrTa
WORKDIR /home/colcon_ws/src/social_navigation/social_navigation_py/social_navigation_py/SMrTa
RUN pip3 install -r requirements.txt
RUN pip3 install .
WORKDIR /home/colcon_ws/src/social_navigation/social_navigation_py/social_navigation_py/SMrTa/bitwuzla
RUN pip3 install .
RUN echo "export PYTHONPATH=\$PYTHONPATH:$(pwd)/build/src/api/python" >> ~/.bashrc

WORKDIR /home/colcon_ws
#RUN add-apt-repository ppa:deadsnakes/ppa
#RUN apt-get install -y python3.11
#RUN apt-get install -y python3.11-distutils
#RUN apt-get install -y python3.11-dev
#RUN apt-get install -y curl
#RUN curl -sS https://bootstrap.pypa.io/get-pip.py | python3.11

#RUN python3.11 -m pip install numpy matplotlib
#RUN python3.11 -m pip install --upgrade "jax[cuda11_pip]" -f https://storage.googleapis.com/jax-releases/jax_cuda_releases.html jaxlib



#RUN apt-get install -y ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-turtlebot3*
#RUN apt-get install -y ros-humble-nav2-simple-commander
#RUN pip3 install --upgrade setuptools
#RUN pip3 install cvxpy jax jaxlib testresources cvxpylayers
#RUN pip3 install --upgrade numpy
#RUN pip3 install gurobipy
#RUN python3 -m pip install jaxopt
#RUN pip3 install -U setuptools
#RUN pip3 install polytope
#WORKDIR /home/
#RUN echo "export PYTHONPATH=$PYTHONPATH:/home/colcon_ws/src/social_navigation/src" >> ~/.bashrc
#RUN echo "source /opt/ros/galactic/setup.bash" >> ~/.bashrc
#RUN echo "source /usr/share/gazebo/setup.sh" >> ~/.bashrc
#RUN echo "source /home/colcon_ws/install/local_setup.bash" >> ~/.bashrc

RUN echo "alias rgazebo='ros2 launch aws_robomaker_hospital_world view_hospital.launch.py'" >> ~/.bashrc
RUN echo "alias rnav2='ros2 launch social_navigation nav2_tb3_aws_launch.py'" >> ~/.bashrc
RUN echo "alias rcsetup='ros2 launch social_navigation init_controller_setup.launch.py'" >> ~/.bashrc
RUN echo "alias rcpub='ros2 topic pub /planner_init std_msgs/msg/Bool data:\ true'" >> ~/.bashrc
RUN echo "alias rcbf='/home/colcon_ws/src/social_navigation/social_navigation_py/social_navigation_py/multiagent_nav2_cbf.sh'" >> ~/.bashrc
RUN echo "alias rcbf2r1='ros2 run social_navigation_py nav2_cbf --ros-args -p use_sim_time:=True -r robot_controller:__node:=robot_controller_1 -r basic_navigator:__node:=basic_navigator -p "robot_name:='robot1'" -p "robot_list:=['robot2']"'" >> ~/.bashrc
RUN echo "alias multi_rcbf='ros2 launch social_navigation multi_cbf.launch.py'" >> ~/.bashrc
RUN echo "alias rmppi='ros2 run social_navigation_py nav2_mppi --ros-args -p use_sim_time:=True'" >> ~/.bashrc
RUN echo "alias rsfm='ros2 launch social_navigation human_sfm.launch.py'" >> ~/.bashrc
RUN echo "alias rcset='ros2 launch social_navigation goal_setter.launch.py'" >> ~/.bashrc
RUN echo "alias rcdis='ros2 run social_navigation_py dispatcher --ros-args -p use_sim_time:=True'" >> ~/.bashrc
# RUN echo "alias rcdis='ros2 launch social_navigation dispatcher.launch.py'" >> ~/.bashrc
RUN echo "alias rctimecollect='ros2 run social_navigation_py goal_setter_for_travel_time --ros-args -p use_sim_time:=True'" >> ~/.bashrc
RUN echo "alias rcplan='ros2 run social_navigation_py planner_wrapper --ros-args -p use_sim_time:=True'" >> ~/.bashrc
RUN echo "alias rqueues='ros2 run social_navigation_py room_queue'" >> ~/.bashrc
