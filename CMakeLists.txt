cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(pcl_hole_analysis)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (pcl_hole_analysis pcl_hole_analysis.cpp)
target_link_libraries (pcl_hole_analysis ${PCL_LIBRARIES})