FROM hardikparwana/cuda116desktop:ipopt-ros
RUN apt-get update && apt-get -y upgrade
RUN apt-get install -y ros-galactic-navigation2 ros-galactic-nav2-bringup ros-galactic-turtlebot3*
RUN echo "source /opt/ros/galactic/setup.bash" >> ~/.bashrc
RUN echo "source /usr/share/gazebo/setup.sh" >> ~/.bashrc
RUN echo "source /home/colcon_ws/install/local_setup.bash" >> ~/.bashrc
WORKDIR /home/
