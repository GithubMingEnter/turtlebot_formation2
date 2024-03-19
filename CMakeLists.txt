cmake_minimum_required(VERSION 3.0.2)
project(turtlebot_formation2)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
set(CMAKE_BUILD_TYPE "Debug")
find_package(catkin REQUIRED COMPONENTS
stage_ros
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  std_msgs
  sensor_msgs
  pcl_ros
  tf
  laser_geometry
)

add_message_files(
  FILES
  avoid.msg
  position.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs   
  nav_msgs   
  std_msgs
)
include_directories(
  include
  src/nmpc_ctrl
  src/esdf_map
  include/${PROJECT_NAME}
  ${catkin_INCLUDE_DIRS}
)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES stage_first
  CATKIN_DEPENDS stage stage_ros roscpp std_msgs geometry_msgs sensor_msgs nav_msgs
#  DEPENDS system_lib
)
include_directories(include ${catkin_INCLUDE_DIRS})

add_library(fusion_esdf src/esdf_map/fusion_esdf.cc)
target_link_libraries(fusion_esdf ${catkin_LIBRARIES})


add_executable(leader src/leader.cpp)
target_link_libraries(leader ${catkin_LIBRARIES})


add_executable(leader1 src/leader1.cpp)
target_link_libraries(leader1 ${catkin_LIBRARIES})

add_executable(tf_broadcaster src/tf_broadcaster.cpp)
target_link_libraries(tf_broadcaster ${catkin_LIBRARIES})

add_executable(slave_tf_listener src/slave_tf_listener.cpp)
add_dependencies(slave_tf_listener ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(slave_tf_listener ${catkin_LIBRARIES})


add_executable(multi_avoidance src/multi_avoidance.cpp)
add_dependencies(multi_avoidance ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(multi_avoidance ${catkin_LIBRARIES})



add_executable(slave_avoid src/slave_avoid.cpp)
add_dependencies(slave_avoid ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(slave_avoid ${catkin_LIBRARIES})

add_executable(slave_tf_listener2 src/slave_tf_listener2.cpp)
target_link_libraries(slave_tf_listener2 ${catkin_LIBRARIES})

add_executable(slave_tf_listener3 src/slave_tf_listener3.cpp)
target_link_libraries(slave_tf_listener3 ${catkin_LIBRARIES})

add_executable(nmpc_leader src/nmpc_ctrl/nmpc_leader.cc src/nmpc_ctrl/nmpc_ctrl.cc)
target_link_libraries(nmpc_leader ${catkin_LIBRARIES} casadi fusion_esdf)


add_executable(fusion_esdf_node src/esdf_map/fusion_esdf_node.cc 
                              src/esdf_map/fusion_esdf.cc )
target_link_libraries(fusion_esdf_node ${catkin_LIBRARIES})


                              ## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)



