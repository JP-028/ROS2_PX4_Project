FROM osrf/ros:humble-desktop

ARG DEBIAN_FRONTEND=noninteractive

USER root

# Basic development and repository tools
RUN apt-get update -o Acquire::Retries=5 && \
    apt-get install -y --no-install-recommends \
        build-essential \
        ca-certificates \
        cmake \
        curl \
        git \
        gnupg \
        lsb-release \
        nano \
        ninja-build \
        python3-colcon-common-extensions \
        python3-pip \
        python3-rosdep \
        python3-vcstool \
        sudo \
        wget && \
    rm -rf /var/lib/apt/lists/*

# Gazebo Harmonic package repository
RUN curl -fsSL \
        https://packages.osrfoundation.org/gazebo.gpg \
        -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
    echo "deb [arch=amd64 signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
http://packages.osrfoundation.org/gazebo/ubuntu-stable jammy main" \
        > /etc/apt/sources.list.d/gazebo-stable.list

# Gazebo, ROS-Gazebo bridge, ROS runtime dependencies and build libraries
RUN apt-get update -o Acquire::Retries=5 && \
    apt-get install -y --no-install-recommends \
        gz-harmonic \
        libasio-dev \
        libpcre2-dev \
        libssl-dev \
        libtinyxml2-dev \
        python3-jinja2 \
        python3-numpy \
        python3-yaml \
        ros-humble-ros-gzharmonic-bridge \
        ros-humble-rmw-fastrtps-cpp \
        ros-humble-tf2-geometry-msgs \
        ros-humble-tf2-ros \
        swig && \
    rm -rf /var/lib/apt/lists/*

# Non-root development user
RUN useradd -ms /bin/bash user && \
    echo "user ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers && \
    mkdir -p /home/user/ros2_ws/src && \
    chown -R user:user /home/user

USER user
WORKDIR /home/user/ros2_ws

CMD ["bash"]
