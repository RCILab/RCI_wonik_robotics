cmake_minimum_required(VERSION 3.0.2)
project(wonik_navigation)

find_package(catkin REQUIRED)
catkin_package()

catkin_install_python(PROGRAMS scripts/route_supervisor.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY config launch maps rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
