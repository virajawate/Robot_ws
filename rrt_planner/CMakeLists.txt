cmake_minimum_required(VERSION 3.0.2)
project(rrt_planner)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  costmap_2d
  geometry_msgs
  nav_core
  nav_msgs
  navfn
  pluginlib
  sensor_msgs
  roscpp
  std_msgs
  tf
  base_local_planner
)

# find_package(OMPL PATHS /opt/ros/noetic/include/ompl-1.6/)
find_package(OMPL)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES rrt_planner_lib
 CATKIN_DEPENDS roscpp costmap_2d geometry_msgs nav_core nav_msgs ompl pluginlib sensor_msgs std_msgs tf
)

include_directories(
  include()  
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(rrt_planner_lib  src/rrt_planner.cpp)

target_link_libraries(rrt_planner_lib  ${OMPL_LIBRARIES})

add_dependencies(rrt_planner_lib ${${PROJECT_NAME}_gencfg} ${catkin_EXPORTED_TARGETS})

add_executable(${PROJECT_NAME} src/rrt_planner.cpp)

#############
## Install ##
#############

install(TARGETS rrt_planner_lib
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(FILES rrt_planner_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)