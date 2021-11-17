cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(bag-to-pcd)

find_package(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (bag_to_pcd, bag_to_pcd.cpp)
target_link_libraries (bag_to_pcd ${PCL_LIBRARIES})
