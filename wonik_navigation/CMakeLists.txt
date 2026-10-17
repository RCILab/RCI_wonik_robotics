cmake_minimum_required(VERSION 3.0.2)
project(wonik_navigation)

find_package(catkin REQUIRED)
catkin_python_setup()
catkin_package()

catkin_install_python(PROGRAMS
  scripts/cmd_vel_gate.py
  scripts/obstacle_detector.py
  scripts/route_supervisor.py
  scripts/zone_manager.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY config launch maps rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
