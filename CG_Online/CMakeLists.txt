cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(CG_Online)
find_package(PCL 1.3 REQUIRED COMPONENTS common io)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
find_package( 
OpenCV REQUIRED )
find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
)
#find_package( Eigen3 REQUIRED )
catkin_package()

set(EIGEN3_INCLUDE_DIR "/usr/include/eigen3")
set(OpenNI_INCLUDE_DIR "/usr/include/ni")
set(PCL_INCLUDE_DIRS "/usr/local/pcl-1.7")
#set(PCL_INCLUDE_DIRS "/usr/local/include/pcl-1.8")
set(PCL_LIBRARY_DIRS "/usr/lib")

include_directories(${PCL_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR} ${OpenNI_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS})
#add_library(shot.o STATIC shot.cpp)
#set_target_properties(shot.o PROPERTIES
#     LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
add_definitions(${PCL_DIFINITIONS})
set(SOURCE
    ${CMAKE_CURRENT_SOURCE_DIR}/src/shot.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/randutil.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/utils.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/MeshPointsIndex.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/TrainMain.cpp
  )
set(RS_SOURCE
    ${CMAKE_CURRENT_SOURCE_DIR}/src/shot.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/randutil.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/utils.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/MeshPointsIndex.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp
  )
#add_executable(TrainMain ${SOURCE})
#target_link_libraries(TrainMain libpcl_search.so libpcl_visualization.so libpcl_segmentation.so libpcl_features.so libpcl_filters.so libgomp.so libpcl_keypoints.so libpcl_kdtree.so libpcl_recognition.so ${PCL_LIBRARIES} ${catkin_LIBRARIES})

add_executable(RS_TrainMain ${RS_SOURCE})
target_link_libraries(RS_TrainMain libpcl_search.so libpcl_visualization.so libpcl_segmentation.so libpcl_features.so libpcl_filters.so libgomp.so libpcl_keypoints.so libpcl_kdtree.so libpcl_recognition.so ${PCL_LIBRARIES} ${catkin_LIBRARIES})

