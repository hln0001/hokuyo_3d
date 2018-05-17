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
  dynamixel_sdk
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES dynamixel_sdk
#  CATKIN_DEPENDS roscpp rospy std_msgs tf2 urg_node
#  DEPENDS system_lib
)

include_directories(
  include/dynamixel_sdk
  ${catkin_INCLUDE_DIRS}
)

add_executable(laserscan_to_pointcloud src/laserscan_to_pointcloud.cpp)
target_link_libraries(laserscan_to_pointcloud ${catkin_LIBRARIES})

#add_executable(motor_sweep_half src/motor_sweep_half.cpp)
#target_link_libraries(motor_sweep_half ${catkin_LIBRARIES})

add_executable(laser_transform_broadcaster src/laser_transform_broadcaster.cpp)
target_link_libraries(laser_transform_broadcaster ${catkin_LIBRARIES})

add_executable(dynamixel_spin_test src/dynamixel_spin_test.cpp)
target_link_libraries(dynamixel_spin_test ${catkin_LIBRARIES})

add_executable(dynamixel_rotate_test src/dynamixel_rotate_test.cpp)
target_link_libraries(dynamixel_rotate_test ${catkin_LIBRARIES})


#add_executable(node executable)
#set_target_properties(node ${catkin_LIBRARIES})

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
