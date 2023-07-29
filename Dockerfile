FROM hardikparwana/cuda118desktop:ros-humble-rmf
RUN apt-get upgrade
RUN wget https://ompl.kavrakilab.org/install-ompl-ubuntu.sh
RUN chmod +x install-ompl-ubuntu.sh
RUN ./install-ompl-ubuntu.sh
RUN apt install ros-humble-test-msgs ros-humble-hardware-interface ros-humble-ros2-control ros-humble-realtime-tools ros-humble-behaviortree-cpp-v3 graphicsmagick ros-humble-diagnostic-updater libgraphicsmagick++1-dev xtensor-dev ros-humble-gazebo-ros-pkgs
#ros-humble-control-msgs ros-humble-bondcpp
#RUN apt-get install -y ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-turtlebot3-gazebo



# Python3.9 for CrowdNav
#RUN add-apt-repository ppa:deadsnakes/ppa
#RUN apt-get install -y python3.9 python3.9-dev python3.9-distutils
#RUN python3.9 -m pip install --upgrade pip setuptools==57.5.0
#RUN python3.9 -m pip install Cython cmake wheel==0.38.4
#RUN apt install libjpeg-dev zlib1g-dev
#RUN python3.9 -m pip install gym==0.18.0

#WORKDIR /home/
#RUN git clone https://github.com/sybrenstuvel/Python-RVO2.git
#RUN git clone https://github.com/vita-epfl/CrowdNav.git
#WORKDIR /home/Python-RVO2
#RUN python3.9 setup.py build
#RUN python3.9 setup.py install
#WORKDIR /home/CrowdNav
#RUN python3.9 -m pip install -e .
#RUN python3.9 -m pip install numpy==1.25.1 matplotlib==3.7.2 Pillow==7.2.0 kiwisolver==1.4.4

#COPY catkin_ws/src/dependencies/CrowdNav /home/CrowdNav
#WORKDIR /home/CrowdNav
#RUN python3.9 -m pip install -e .

WORKDIR /home/hsr
