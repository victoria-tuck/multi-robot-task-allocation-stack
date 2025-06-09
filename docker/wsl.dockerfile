# WSL2-optimized version
# Note: For CUDA support in WSL2, ensure you have WSL2 CUDA drivers installed on Windows host
FROM hardikparwana/cuda118desktop:ros-humble-rmf

# WSL2-specific environment variables
ENV DISPLAY=:0
ENV LIBGL_ALWAYS_INDIRECT=0
ENV WSL_DISTRO_NAME=Ubuntu

RUN rm /var/lib/apt/lists/*ros*
RUN sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg 
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu \
     $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Combine RUN commands to reduce layers (better for WSL2 performance)
RUN apt-get update && apt-get install -y \
    wget \
    build-essential \
    libncursesw5-dev \
    libssl-dev \
    libsqlite3-dev \
    tk-dev \
    libgdbm-dev \
    libc6-dev \
    libbz2-dev \
    libffi-dev \
    zlib1g-dev \
    software-properties-common \
    curl \
    python3.10-dev \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-turtlebot3* \
    ros-humble-nav2-simple-commander \ 
    ros-humble-rqt-tf-tree \
    ros-humble-topic-tools \
    && rm -rf /var/lib/apt/lists/* 

# Python setup
RUN curl -sS https://bootstrap.pypa.io/get-pip.py | python3.10
RUN python3 -m pip install numpy

# Python packages installation (combined for efficiency)
RUN pip3 install -U setuptools && \
    pip3 install polytope numpy cvxpy jax jaxlib testresources cvxpylayers gurobipy

RUN python3 -m pip install setuptools==58.2.0 && \
    python3 -m pip install numpy==1.26.4 matplotlib

# JAX with CUDA support (ensure WSL2 CUDA drivers are installed)
RUN python3 -m pip install --upgrade "jax[cuda11_pip]==0.4.25" -f https://storage.googleapis.com/jax-releases/jax_cuda_releases.html jaxlib==0.4.25

RUN python3 -m pip install matplotlib==3.7.1 pillow==9.5.0 kiwisolver==1.4.4 polytope && \
    python3 -m pip install myst-parser sphinx sphinx-rtd-theme

# Environment setup (WSL2-friendly bashrc modifications)
RUN echo "export PYTHONPATH=\$PYTHONPATH:/home/colcon_ws/src/social_navigation/src" >> ~/.bashrc && \
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /usr/share/gazebo/setup.sh" >> ~/.bashrc && \
    echo "source /home/colcon_ws/install/local_setup.bash" >> ~/.bashrc && \
    echo "export DISPLAY=\${DISPLAY:-:0}" >> ~/.bashrc && \
    echo "export LIBGL_ALWAYS_INDIRECT=1" >> ~/.bashrc

WORKDIR /home/

# Build lightsfm
RUN git clone https://github.com/robotics-upo/lightsfm.git
WORKDIR /home/lightsfm
RUN make && make install

# Setup SMrTa
ADD colcon_ws/src/social_navigation/social_navigation_py/social_navigation_py/SMrTa /home/colcon_ws/src/social_navigation/social_navigation_py/social_navigation_py/SMrTa
WORKDIR /home/colcon_ws/src/social_navigation/social_navigation_py/social_navigation_py/SMrTa
RUN pip3 install -r requirements.txt && pip3 install .

WORKDIR /home/colcon_ws/src/social_navigation/social_navigation_py/social_navigation_py/SMrTa/bitwuzla
RUN pip3 install . && \
    echo "export PYTHONPATH=\$PYTHONPATH:$(pwd)/build/src/api/python" >> ~/.bashrc

WORKDIR /home/colcon_ws

# Aliases (unchanged as they work fine in WSL2)
RUN echo "alias rgazebo='ros2 launch aws_robomaker_hospital_world view_hospital.launch.py'" >> ~/.bashrc && \
    echo "alias rnav2='ros2 launch social_navigation nav2_tb3_aws_launch.py'" >> ~/.bashrc && \
    echo "alias rcsetup='ros2 launch social_navigation init_controller_setup.launch.py'" >> ~/.bashrc && \
    echo "alias rcpub='ros2 topic pub /planner_init std_msgs/msg/Bool data:\ true'" >> ~/.bashrc && \
    echo "alias rcbf='/home/colcon_ws/src/social_navigation/social_navigation_py/social_navigation_py/multiagent_nav2_cbf.sh'" >> ~/.bashrc && \
    echo "alias rcbf2r1='ros2 run social_navigation_py nav2_cbf --ros-args -p use_sim_time:=True -r robot_controller:__node:=robot_controller_1 -r basic_navigator:__node:=basic_navigator -p \"robot_name:='robot1'\" -p \"robot_list:=['robot2']\"'" >> ~/.bashrc && \
    echo "alias multi_rcbf='ros2 launch social_navigation multi_cbf.launch.py'" >> ~/.bashrc && \
    echo "alias rmppi='ros2 run social_navigation_py nav2_mppi --ros-args -p use_sim_time:=True'" >> ~/.bashrc && \
    echo "alias rsfm='ros2 launch social_navigation human_sfm.launch.py'" >> ~/.bashrc && \
    echo "alias rcset='ros2 launch social_navigation goal_setter.launch.py'" >> ~/.bashrc && \
    echo "alias rcdis='ros2 run social_navigation_py dispatcher --ros-args -p use_sim_time:=True'" >> ~/.bashrc && \
    echo "alias rctimecollect='ros2 run social_navigation_py goal_setter_for_travel_time --ros-args -p use_sim_time:=True'" >> ~/.bashrc && \
    echo "alias rcplan='ros2 run social_navigation_py planner_wrapper --ros-args -p use_sim_time:=True'" >> ~/.bashrc && \
    echo "alias rqueues='ros2 run social_navigation_py room_queue'" >> ~/.bashrc

