cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(large_point_cloud_preprocessing)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (large_point_cloud_preprocessing large_point_cloud_preprocessing.cpp)
target_link_libraries (large_point_cloud_preprocessing ${PCL_LIBRARIES})