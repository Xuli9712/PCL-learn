cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(create_range_image)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (create_range_image create_range_image.cpp)
target_link_libraries (create_range_image ${PCL_LIBRARIES})

set(CMAKE_BUILD_TYPE DEBUG) 
