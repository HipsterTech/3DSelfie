cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(3DSelfie)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (3DSelfie 3DSelfie.cpp)
target_link_libraries (3DSelfie ${PCL_LIBRARIES})
