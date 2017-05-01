cmake_minimum_required(VERSION 2.8.3)
project(my_pcl_tutorial)

find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
)
catkin_package(
#  INCLUDE_DIRS include
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(cloud_viewer src/cloud_viewer.cpp)
target_link_libraries (cloud_viewer ${catkin_LIBRARIES})

add_executable(example src/example.cpp)
target_link_libraries(example ${catkin_LIBRARIES})

add_executable(realsense_viewer src/realsense_viewer.cpp)
target_link_libraries(realsense_viewer ${catkin_LIBRARIES})

add_executable(correspondence_grouping src/correspondence_grouping.cpp)
target_link_libraries(correspondence_grouping ${catkin_LIBRARIES})
