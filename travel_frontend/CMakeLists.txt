cmake_minimum_required(VERSION 3.0.2)
project(travel_frontend)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  grid_map_msgs
  grid_map_ros
  pcl_ros
  roscpp
  sensor_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES travel_frontend
#  CATKIN_DEPENDS grid_map_msgs grid_map_ros pcl_ros roscpp sensor_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(travel_frontend src/travel_frontend.cpp)
target_link_libraries(travel_frontend ${catkin_LIBRARIES})
