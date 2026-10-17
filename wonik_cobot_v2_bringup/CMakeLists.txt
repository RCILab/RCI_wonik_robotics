cmake_minimum_required(VERSION 3.0.2)
project(wonik_cobot_v2_bringup)

find_package(catkin REQUIRED)
catkin_package()

install(DIRECTORY config launch rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
