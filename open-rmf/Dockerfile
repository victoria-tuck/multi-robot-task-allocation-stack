FROM nvidia/cuda:11.8.0-cudnn8-devel-ubuntu22.04
RUN apt-get update 
RUN apt-get install -y tmux gedit
RUN apt-get update
RUN apt install -y python3-pip
RUN apt install -y software-properties-common
RUN add-apt-repository universe
RUN apt update && apt install curl -y
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null
RUN apt update
RUN apt -y upgrade
#RUN apt install -y ros-humble-desktop-full
#RUN apt install -y ros-dev-tools
#RUN echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
