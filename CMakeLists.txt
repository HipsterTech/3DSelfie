cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(3DSelfie)

if(UNIX)
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=gnu++0x")
endif()

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})
if(APPLE)
	add_executable (3DSelfie MACOSX_BUNDLE 3DSelfie.cpp)
else()
	add_executable (3DSelfie  3DSelfie.cpp)
endif()
target_link_libraries (3DSelfie ${PCL_LIBRARIES})
