cmake_minimum_required(VERSION 2.8.3)
project(global_fusion)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
#-DEIGEN_USE_MKL_ALL")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf2
  tf2_ros
  geometry_msgs
  nav_msgs
  tf
  # pcl
  pcl_conversions
)

find_package(OpenCV 3.4.1 REQUIRED PATHS "/storage_ssd/opencv-3.4.1" NO_DEFAULT_PATH)
find_package(Ceres REQUIRED)
find_package(PCL REQUIRED)

add_subdirectory(./ThirdParty/GeographicLib/)

include_directories( 
  ${PCL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ./ThirdParty/GeographicLib/include/
)


catkin_package()

add_library(vinsg_lib
    src/parameters.cpp)
target_link_libraries(vinsg_lib ${OpenCV_LIBS} ${catkin_LIBRARIES}  ${CERES_LIBRARIES} ${PCL_LIBRARIES})


add_executable(global_fusion_node 
	src/globalOptNode.cpp
	src/globalOpt.cpp)

target_link_libraries(global_fusion_node vinsg_lib libGeographiccc) 
