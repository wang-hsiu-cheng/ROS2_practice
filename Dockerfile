FROM osrf/ros:humble-desktop-full

# each RUN makes the image size bigger
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y \
    git \
    cmake \
    ros-humble-ament-cmake \
    # ros-humble-gazebo-* \
    # ros-humble-rviz2 \
    # ros-humble-laser-geometry \
    # libarmadillo-dev \
    # libboost-all-dev \
    ros-humble-dynamixel-sdk \
    ros-humble-turtlebot3-msgs \
    ros-humble-turtlebot3 \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-rosbridge-server \
    && rm -rf /var/lib/apt/lists/*

RUN echo "source /opt/ros/humble/setup.bash" >> /home/$USERNAME/.bashrc 

# mkdir 
WORKDIR /root/folder

ENV SHELL /bin/bash
SHELL ["/bin/bash","-ic"]