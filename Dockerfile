FROM ros:humble-ros-base

RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    git \
    nano \
    usbutils \
    udev \
    build-essential \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /root/ros2_ws

COPY . src/Project_Security_Robot

RUN apt-get update && rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y \
    --skip-keys "gazebo_ros gazebo_plugins gazebo_msgs"

RUN . /opt/ros/humble/setup.sh && \
    colcon build --symlink-install

RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc
RUN echo "source /root/ros2_ws/install/setup.bash" >> /root/.bashrc

CMD ["bash"]