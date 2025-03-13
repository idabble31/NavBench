cmake_minimum_required(VERSION 3.0.2)
project(lrf_global_planner)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  pluginlib
  nav_core
  costmap_2d
  geometry_msgs
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES lrf_global_planner
  CATKIN_DEPENDS roscpp pluginlib nav_core costmap_2d nav_msgs geometry_msgs
  # DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(lrf_global_planner
  src/dijkstra_planner.cpp
)

target_link_libraries(lrf_global_planner
  ${catkin_LIBRARIES}
)