FROM osrf/ros:noetic-desktop-full

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    git \
    python3-vcstool \
    python3-rosdep \
    python3-catkin-tools \
    ros-noetic-gazebo-ros-pkgs \
    ros-noetic-gazebo-plugins \
    libopencv-dev \
    libpcl-dev \
    build-essential \
    cmake \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep init || true

WORKDIR /workspaces/dave_ws

COPY scripts/build_multibeam.sh /usr/local/bin/build_multibeam.sh
RUN chmod +x /usr/local/bin/build_multibeam.sh

CMD ["bash"]
