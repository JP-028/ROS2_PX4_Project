FROM osrf/ros:humble-desktop

ARG DEBIAN_FRONTEND=noninteractive

RUN rm -rf /var/lib/apt/lists/* && \
    apt-get clean && \
    apt-get update -o Acquire::Retries=5 && \
    apt-get install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    git \
    build-essential \
    sudo \
    nano \
    wget && \
    rm -rf /var/lib/apt/lists/*

RUN useradd -ms /bin/bash user && echo "user ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

USER user
WORKDIR /home/user/ros2_ws

CMD ["bash"]