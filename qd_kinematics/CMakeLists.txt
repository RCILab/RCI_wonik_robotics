cmake_minimum_required(VERSION 3.0.2)
project(qd_kinematics)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}_core
  CATKIN_DEPENDS roscpp geometry_msgs nav_msgs sensor_msgs std_msgs tf2 tf2_ros tf2_geometry_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})

# ROS-independent kinematics core (identical to the ROS 2 version).
add_library(${PROJECT_NAME}_core
  src/kinematics_model.cpp
  src/steer_drive_controller.cpp
  src/odometry_integrator.cpp
)

add_executable(kinematics_node src/kinematics_node.cpp)
target_link_libraries(kinematics_node ${PROJECT_NAME}_core ${catkin_LIBRARIES})
add_dependencies(kinematics_node ${catkin_EXPORTED_TARGETS})

install(TARGETS ${PROJECT_NAME}_core kinematics_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_kinematics_core test/test_kinematics_core.cpp)
  # The test file is shared with the ROS 2 package, where ament links gtest_main.
  target_link_libraries(test_kinematics_core ${PROJECT_NAME}_core ${GTEST_MAIN_LIBRARIES})
endif()
