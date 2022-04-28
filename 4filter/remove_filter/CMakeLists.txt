cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(remove_filter)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (remove_filter remove_filter.cpp)
target_link_libraries (remove_filter ${PCL_LIBRARIES})