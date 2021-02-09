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
  biotac_interface
  emg_interface
  map_msgs
  # jsk_recognition_msgs
  # geometric_shapes_msgs
  # pr2_arm_navigation_tutorials
)

find_package(Eigen3 REQUIRED eigen3) 


catkin_package(
 INCLUDE_DIRS include
 LIBRARIES ${PROJECT_NAME}
 CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs
#  DEPENDS system_lib
)

# find_package(emg_interface)

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

add_executable(fingers_trq_controller src/ft_torque_controller.cpp)
target_include_directories(fingers_trq_controller PUBLIC include ${catkin_INCLUDE_DIRS} ${Eigen3_INCLUDE_DIRS})
target_link_libraries(fingers_trq_controller ${catkin_LIBRARIES} ${Eigen3_LIBRARIES})
add_dependencies(fingers_trq_controller ${PROJECT_NAME}_gencpp)

add_executable(obj_motion_gen_node src/obj_motion_gen_node.cpp)
target_include_directories(obj_motion_gen_node PUBLIC include ${catkin_INCLUDE_DIRS} ${Eigen3_INCLUDE_DIRS})
target_link_libraries(obj_motion_gen_node ${catkin_LIBRARIES} ${Eigen3_LIBRARIES})
add_dependencies(obj_motion_gen_node ${PROJECT_NAME}_gencpp)

add_executable(fbft_controller src/fingers_biotac_ft.cpp)
target_include_directories(fbft_controller PUBLIC include ${catkin_INCLUDE_DIRS} ${Eigen3_INCLUDE_DIRS})
target_link_libraries(fbft_controller ${catkin_LIBRARIES} ${Eigen3_LIBRARIES})
add_dependencies(fbft_controller ${PROJECT_NAME}_gencpp)

add_executable(prop_rot_controller src/fnPropRot.cpp)
target_include_directories(prop_rot_controller PUBLIC include ${catkin_INCLUDE_DIRS} ${Eigen3_INCLUDE_DIRS})
target_link_libraries(prop_rot_controller ${catkin_LIBRARIES} ${Eigen3_LIBRARIES})
add_dependencies(prop_rot_controller ${PROJECT_NAME}_gencpp)

add_executable(test_prop src/test_prop_controller.cpp)
target_include_directories(test_prop PUBLIC include ${catkin_INCLUDE_DIRS} ${Eigen3_INCLUDE_DIRS})
target_link_libraries(test_prop ${catkin_LIBRARIES} ${Eigen3_LIBRARIES})
add_dependencies(test_prop ${PROJECT_NAME}_gencpp)