cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_INSTALL_PREFIX /usr/bin)

project(large_point_cloud_preprocessing)

find_package(PCL 1.10 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
include_directories(include)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (large_point_cloud_preprocessing main.cpp include/preprocessing/filters.h include/preprocessing/normal_estimation.h include/preprocessing/normalization.h)
target_link_libraries (large_point_cloud_preprocessing ${PCL_LIBRARIES})

install(TARGETS large_point_cloud_preprocessing
    DESTINATION ${CMAKE_INSTALL_PREFIX}
    )