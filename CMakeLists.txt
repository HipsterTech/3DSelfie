cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)


project(3DSelfie)

if(UNIX)
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=gnu++0x")
endif()

find_package(PCL 1.2 REQUIRED)
find_package(OpenCV REQUIRED core highgui imgproc xfeatures2d)

include_directories(${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})




add_executable (3DSelfie src/3DSelfie.cpp)
target_link_libraries (3DSelfie ${PCL_LIBRARIES} ${OpenCV_LIBS})

#add_executable (ht_icp_check src/tools/ht_icp_check.cpp)
#target_link_libraries (ht_icp_check ${PCL_LIBRARIES} ${PCL_IO_LIBRARIES})
