FROM osrf/ros:humble-desktop-full

RUN apt update && apt install -y \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros2-control \
    ros-humble-xacro \
    ros-humble-rviz2 \
    && rm -rf /var/lib/apt/lists/*

# 下载 VRX 模型（海面、浮标等）
RUN mkdir -p /gazebo_models && \
    cd /gazebo_models && \
    wget -q https://github.com/osrf/vrx/archive/humble.tar.gz -O vrx.tar.gz && \
    tar -xzf vrx.tar.gz --strip-components=2 vrx-humble/models && \
    rm vrx.tar.gz

ENV GAZEBO_MODEL_PATH=/gazebo_models:${GAZEBO_MODEL_PATH}

# 复制源码并编译
COPY ./src /ws/src
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && cd /ws && colcon build"

# 启动命令占位
CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && source /ws/install/setup.bash && ros2 launch usv_bringup sim.launch.py"]
