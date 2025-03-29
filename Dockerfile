FROM osrf/ros:jazzy-desktop

ENV DEBIAN_FRONTEND=noninteractive
ENV XDG_RUNTIME_DIR=/tmp/runtime-root

RUN apt update && apt install -y \
    build-essential \
    vim \
    python3-pip \
    git \
    libgl1-mesa-dev \
    libgl1-mesa-glx \
    libglu1-mesa \
    mesa-utils \
    x11-xserver-utils \
    ros-jazzy-slam-toolbox \
    ros-jazzy-xacro \
    ros-jazzy-robot-state-publisher \
    && rm -rf /var/lib/apt/lists/*

RUN mkdir -p /tmp/runtime-root && chmod 700 /tmp/runtime-root

WORKDIR /home/ros2_ws

RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc

CMD ["bash"]