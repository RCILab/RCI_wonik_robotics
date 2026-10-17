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

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  catkin_add_nosetests(test/test_safety.py)
  add_rostest(test/gate_timeouts.test)
  add_rostest(test/supervisor_gate_timeout.test)
  add_rostest(test/supervisor_gate_fail.test)
  add_rostest(test/supervisor_gate_rejected.test)
  add_rostest(test/supervisor_gate_ok.test)
endif()
