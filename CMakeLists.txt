cmake_minimum_required(VERSION 2.8.3)
project(fingers_controller)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
)

find_package(Eigen3 REQUIRED eigen3) 


catkin_package(
 INCLUDE_DIRS include
 LIBRARIES ${PROJECT_NAME}
 CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs
#  DEPENDS system_lib
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

include_directories(${EIGEN3_INCLUDE_DIR} ${RBDL_INCLUDE_DIR} )


add_executable(fingers_controller src/finger_controller.cpp)
target_include_directories(fingers_controller PUBLIC include ${catkin_INCLUDE_DIRS} ${Eigen3_INCLUDE_DIRS})
target_link_libraries(fingers_controller ${catkin_LIBRARIES} ${Eigen3_LIBRARIES})
add_dependencies(fingers_controller ${PROJECT_NAME}_gencpp)