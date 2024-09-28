FROM ubuntu:22.04
#adopted from: https://github.com/IntelRealSense/librealsense/blob/v2.51.1/scripts/Docker/Dockerfile

ARG DEBIAN_FRONTEND=noninteractive

# Builder dependencies installation
RUN apt-get update \
    && apt-get install -qq -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    libssl-dev \
    libusb-1.0-0-dev \
    pkg-config \
    libgtk-3-dev \
    libglfw3-dev \
    libgl1-mesa-dev \
    libglu1-mesa-dev \
    curl \
    python3 \
    python3-dev \
    ca-certificates \
    && rm -rf /var/lib/apt/lists/*


FROM osrf/ros:humble-desktop-full

ARG USERNAME=sentient-beings
ARG USER_UID=1000
ARG USER_GID=$USER_UID

ARG DEBIAN_FRONTEND=noninteractive

SHELL ["/bin/bash", "-c"]
ENV SHELL=/bin/bash

RUN apt update; apt install build-essential \
                sudo \
                autoconf \
                git \
                libargon2-dev \
                libssl-dev \
                libx11-dev \
                cmake \
                libgstreamer1.0-dev \
                libgstreamer-plugins-base1.0-dev \
                libgstreamer-plugins-bad1.0-dev \
                libxext-dev \
                flex \
                bison \
                gstreamer1.0-pulseaudio \
                python3-pip \
                ros-humble-gazebo-* \
                ros-humble-cartographer \ 
                ros-humble-cartographer-ros \ 
                ros-humble-navigation2 \ 
                ros-humble-nav2-bringup -y


RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME


RUN usermod -a -G plugdev $USERNAME

USER $USERNAME

RUN pip install --user --upgrade numpy==1.26.2

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source /usr/share/gazebo/setup.sh" >> ~/.bashrc
RUN echo 'export ROS_DOMAIN_ID=30' >> ~/.bashrc
RUN echo 'export TURTLEBOT3_MODEL=waffle' >> ~/.bashrc