FROM osrf/ros:humble-desktop

LABEL org.opencontainers.image.authors="wang.tw.ted@gmail.com"
LABEL shell=bash

ARG USERNAME=user
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    && rm -rf /var/lib/apt/lists/*

# each RUN makes the image size bigger
RUN apt-get update \
    && DEBIAN_FRONTEND=noninteractive apt-get install -y \
    curl \
    git \
    git-extras \
    htop \
    tree \
    net-tools \
    tmux \
    vim \
    wget \
    python3-pip \
    bash-completion \
    ros-humble-ament-cmake \
    ros-humble-rqt-plot\
    ros-humble-rqt-tf-tree\
    # Groot
    libfuse2 \
    fuse3 \
    ros-humble-behaviortree-cpp \
    ros-humble-generate-parameter-library \
    # simulation
    ros-humble-dynamixel-sdk \
    ros-humble-turtlebot3-msgs \
    ros-humble-turtlebot3 \
    ros-humble-gazebo-ros-pkgs \
    # foxglove
    ros-humble-rosbridge-server \
    ros-humble-foxglove-bridge

RUN apt-get update \
    && apt-get dist-upgrade -y \
    && apt-get autoremove -y \
    && apt-get autoclean -y \
    && rm -rf /var/lib/apt/lists/*

RUN git clone git@github.com:BehaviorTree/BehaviorTree.ROS2.git

RUN echo "source /opt/ros/humble/setup.bash" >> /etc/bash.bashrc \ 
    && echo "source /home/Eurobot-2025-Main-ws/install/setup.bash" >> /etc/bash.bashrc

ENV SHELL /bin/bash
ENV TERM=xterm-256color

USER $USERNAME
RUN mkdir -p /home/$USERNAME/groot_ws/groot

COPY .bashrc /home/$USERNAME/.bashrc.conf
RUN cat /home/$USERNAME/.bashrc.conf >> /home/$USERNAME/.bashrc

CMD ["/bin/bash"]