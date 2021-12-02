cmake_minimum_required(VERSION 2.6)
project (cpp_full)

find_package(rosbag REQUIRED
ros_cpp
pcl_conversions
pcl_msgs
pcl_ros
nav_msgs)


find_package(Boost REQUIRED COMPONENTS system filesystem thread)
find_package(PCL REQUIRED)
include_directories(${rosbag_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
add_executable(cpp_full cpp_full.cpp)
link_directories(${Boost_LIBRARY_DIRS} ${PCL_LIBRARY_DIRS})
target_link_libraries(cpp_full ${rosbag_LIBRARIES} ${Boost_LIBRARIES} ${PCL_LIBRARIES})