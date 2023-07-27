FROM hardikparwana/cuda116desktop:ipopt-ros1
RUN sh -c 'echo "deb [arch=amd64] https://hsr-user:jD3k4G2e@packages.hsr.io/ros/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/tmc.list'
RUN sh -c 'echo "deb [arch=amd64] https://hsr-user:jD3k4G2e@packages.hsr.io/tmc/ubuntu `lsb_release -cs` multiverse main" >> /etc/apt/sources.list.d/tmc.list'
RUN sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
RUN wget https://hsr-user:jD3k4G2e@packages.hsr.io/tmc.key -O - | apt-key add -
RUN wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc -O - | apt-key add -
RUN wget https://packages.osrfoundation.org/gazebo.key -O - | apt-key add -
RUN sh -c 'mkdir -p /etc/apt/auth.conf.d'
RUN sh -c '/bin/echo -e "machine packages.hsr.io\nlogin hsr-user\npassword jD3k4G2e" >/etc/apt/auth.conf.d/auth.conf'
RUN sh -c '/bin/echo -e "Package: ros-noetic-laser-ortho-projector\nPin: version 0.3.3*\nPin-Priority: 1001\n\nPackage: ros-noetic-laser-scan-matcher\nPin: version 0.3.3*\nPin-Priority: 1001\n\nPackage: ros-noetic-laser-scan-sparsifier\nPin: version 0.3.3*\nPin-Priority: 1001\n\nPackage: ros-noetic-laser-scan-splitter\nPin: version 0.3.3*\nPin-Priority: 1001\n\nPackage: ros-noetic-ncd-parser\nPin: version 0.3.3*\nPin-Priority: 1001\n\nPackage: ros-noetic-polar-scan-matcher\nPin: version 0.3.3*\nPin-Priority: 1001\n\nPackage: ros-noetic-scan-to-cloud-converter\nPin: version 0.3.3*\nPin-Priority: 1001\n\nPackage: ros-noetic-scan-tools\nPin: version 0.3.3*\nPin-Priority: 1001" > /etc/apt/preferences'
RUN apt-get update
RUN apt-get install -y ros-noetic-tmc-desktop-full
RUN pip3 install notebook
RUN pip3 install jupyterlab==3.5.3
RUN pip3 install ipython==8.0.1
RUN pip3 install casadi==3.6.2 polytope 
RUN apt-get install -y python3-catkin-tools
RUN apt install -y python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
RUN rosdep init
RUN rosdep update
RUN apt install -y libompl-dev
RUN apt install -y ros-noetic-ompl ros-noetic-octomap-rviz-plugins ros-noetic-libg2o

# Install g2o library
WORKDIR /home/
RUN mkdir dependencies
WORKDIR /home/dependencies
RUN git clone https://github.com/RainerKuemmerle/g2o
WORKDIR /home/dependencies/g2o
RUN git checkout tags/20170730_git
RUN mkdir build
WORKDIR /home/dependencies/g2o/build
RUN cmake ..
RUN make
RUN make install

RUN apt install -y ros-noetic-pcl-ros ros-noetic-moveit ros-noetic-robot-controllers ros-noetic-calibration-msgs ros-noetic-simple-grasping

RUN echo "source /home/hsr/catkin_ws/devel/setup.bash" >> ~/.bashrc
RUN apt-get install ros-noetic-teleop-twist-keyboard
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
