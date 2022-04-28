cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(downsample_voxel_grid)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (downsample_voxel_grid downsample_voxel_grid.cpp)
target_link_libraries (downsample_voxel_grid ${PCL_LIBRARIES})