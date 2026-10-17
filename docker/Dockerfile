# ROS 2 Humble, Ubuntu 22.04 and Gazebo Classic 11.
FROM ros:humble-ros-base-jammy
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential python3-colcon-common-extensions python3-pytest python3-numpy python3-yaml \
    ros-humble-ament-cmake-gtest ros-humble-ament-cmake-pytest \
    ros-humble-navigation2 ros-humble-nav2-bringup \
    ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control ros-humble-ros2-controllers \
    ros-humble-robot-state-publisher ros-humble-joint-state-publisher ros-humble-xacro \
    ros-humble-robot-localization ros-humble-pointcloud-to-laserscan \
    ros-humble-slam-toolbox ros-humble-teleop-twist-keyboard ros-humble-tf-transformations \
    ros-humble-rviz2 xvfb \
    && rm -rf /var/lib/apt/lists/*
RUN mkdir -p /ros2_ws/src && \
    echo 'source /opt/ros/humble/setup.bash' >> /root/.bashrc && \
    echo '[ ! -f /ros2_ws/install/setup.bash ] || source /ros2_ws/install/setup.bash' >> /root/.bashrc
WORKDIR /ros2_ws
ENV DISPLAY=:99
COPY command.sh /usr/local/bin/wonik-command
COPY container.sh /usr/local/bin/wonik-container
RUN chmod +x /usr/local/bin/wonik-command /usr/local/bin/wonik-container
CMD ["/usr/local/bin/wonik-container"]
