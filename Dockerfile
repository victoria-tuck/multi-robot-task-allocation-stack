FROM hardikparwana/cuda116desktop:ipopt-ros
RUN apt-get update && apt-get -y upgrade
RUN apt-get install -y ros-galactic-navigation2 ros-galactic-nav2-bringup ros-galactic-turtlebot3*
RUN echo "source /opt/ros/galactic/setup.bash" >> ~/.bashrc
