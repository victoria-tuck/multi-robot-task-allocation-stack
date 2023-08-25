FROM hardikparwana/cuda116desktop:ipopt-ros
RUN apt-get update && apt-get -y upgrade
RUN apt-get install -y ros-galactic-navigation2 ros-galactic-nav2-bringup ros-galactic-turtlebot3*
RUN apt-get install -y ros-galactic-nav2-simple-commander
RUN pip3 install cvxpy jax jaxlib testresources polytope cvxpylayers
RUN pip3 install --upgrade numpy
RUN echo "export PYTHONPATH=$PYTHONPATH:/home/colcon_ws/src/social_navigation/src" >> ~/.bashrc
RUN echo "source /opt/ros/galactic/setup.bash" >> ~/.bashrc
RUN echo "source /usr/share/gazebo/setup.sh" >> ~/.bashrc
RUN echo "source /home/colcon_ws/install/local_setup.bash" >> ~/.bashrc
RUN pip3 install gurobipy
WORKDIR /home/
