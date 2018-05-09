cmake_minimum_required(VERSION 2.8.3)
project(hokuyo_3d)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  message_generation
  pcl_ros
  laser_geometry
  sensor_msgs
  std_msgs
  tf2
  urg_node
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES hokuyo_3d
#  CATKIN_DEPENDS roscpp rospy std_msgs tf2 urg_node
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_executable(motor_sweep src/motor_sweep.cpp)
target_link_libraries(motor_sweep ${catkin_LIBRARIES})

#add_executable(motor_sweep2 src/motor_sweep2.cpp)
#target_link_libraries(motor_sweep2 ${catkin_LIBRARIES})

add_executable(laser_transform_broadcaster src/laser_transform_broadcaster.cpp)
target_link_libraries(laser_transform_broadcaster ${catkin_LIBRARIES})

#add_executable(node executable)
#set_target_properties(node ${catkin_LIBRARIES})

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
