cmake_minimum_required(VERSION 3.0.2)
project(wonik_gazebo)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS gazebo_ros qd_kinematics)
find_package(gazebo REQUIRED)

catkin_package(CATKIN_DEPENDS gazebo_ros qd_kinematics)

include_directories(${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

# Gazebo model plugin: moves the base with qd_kinematics' forward kinematics (sim_drive:=kinematic).
add_library(qd_kinematic_base SHARED src/qd_kinematic_base_plugin.cpp)
target_link_libraries(qd_kinematic_base ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(qd_kinematic_base ${catkin_EXPORTED_TARGETS})

install(TARGETS qd_kinematic_base
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
