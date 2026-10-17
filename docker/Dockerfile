# ROS Noetic + Gazebo 11 environment for RCI_wonik_robotics (Ubuntu 20.04).
# The image holds only the environment. docker/run.sh mounts this repository into the catkin
# workspace /catkin_ws, whose build/ and devel/ live in the Docker volume "rci_wonik_ws".
#   docker build -t rci_wonik_noetic:latest docker
FROM osrf/ros:noetic-desktop-full

RUN echo 'Acquire::Retries "10";' > /etc/apt/apt.conf.d/80-retries \
 && apt-get update && apt-get install -y --no-install-recommends \
      ros-noetic-ros-controllers \
      ros-noetic-gazebo-ros-control \
      ros-noetic-velodyne-simulator \
      ros-noetic-teleop-twist-keyboard \
 && rm -rf /var/lib/apt/lists/*

# 2D SLAM and localization: /points -> /scan -> slam_toolbox, map_server + amcl
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-noetic-pointcloud-to-laserscan \
      ros-noetic-slam-toolbox \
      ros-noetic-map-server \
      ros-noetic-amcl \
 && rm -rf /var/lib/apt/lists/*

RUN mkdir -p /catkin_ws/src \
 && echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc \
 && echo "[ -f /catkin_ws/devel/setup.bash ] && source /catkin_ws/devel/setup.bash" >> /root/.bashrc

WORKDIR /catkin_ws
