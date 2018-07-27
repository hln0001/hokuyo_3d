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
  hokuyo_msgs
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES dynamixel_sdk

  CATKIN_DEPENDS message_runtime
#  CATKIN_DEPENDS roscpp rospy std_msgs tf2 urg_node
#  DEPENDS system_lib
)

include_directories(
  include/dynamixel_sdk
  ${catkin_INCLUDE_DIRS}
)

add_executable(laserscan_to_pointcloud src/laserscan_to_pointcloud.cpp)
target_link_libraries(laserscan_to_pointcloud ${catkin_LIBRARIES})

add_executable(laser_transform_broadcaster src/laser_transform_broadcaster.cpp)
target_link_libraries(laser_transform_broadcaster ${catkin_LIBRARIES})

add_executable(dynamixel_spin_continuous src/dynamixel_spin_continuous.cpp)
target_link_libraries(dynamixel_spin_continuous ${catkin_LIBRARIES})

add_executable(laserscan_ugv_filter src/laserscan_ugv_filter.cpp)
target_link_libraries(laserscan_ugv_filter ${catkin_LIBRARIES})

add_executable(pointcloud_assembler_client src/pointcloud_assembler_client.cpp)
target_link_libraries(pointcloud_assembler_client ${catkin_LIBRARIES})

add_executable(periodic_snapshotter src/periodic_snapshotter.cpp)
target_link_libraries(periodic_snapshotter ${catkin_LIBRARIES})

add_executable(revolution_snapshotter src/revolution_snapshotter.cpp)
target_link_libraries(revolution_snapshotter ${catkin_LIBRARIES})

add_executable(dynamixel_spin_oscillation src/dynamixel_spin_oscillation.cpp)
target_link_libraries(dynamixel_spin_oscillation ${catkin_LIBRARIES})

add_executable(pointcloud_topic_merger src/pointcloud_topic_merger.cpp)
target_link_libraries(pointcloud_topic_merger ${catkin_LIBRARIES})

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
