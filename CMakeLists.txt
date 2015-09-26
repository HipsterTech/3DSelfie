project(3DSelfie)
cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
include(CheckIncludeFileCXX)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)


if(UNIX)
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=gnu++0x")
endif()

#Master check for PCL
find_package(PCL 1.8 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#Be sure that we have the patched tree so that we can use smart file loader
set(CMAKE_REQUIRED_FLAGS ${Boost_SYSTEM_LIBRARY_DEBUG})
set(CMAKE_REQUIRED_INCLUDES ${PCL_INCLUDE_DIRS} ${EIGEN_INCLUDE_DIRS})
CHECK_INCLUDE_FILE_CXX(pcl/io/auto_io.h PCL_HAS_AUTO_IO)
if(NOT PCL_HAS_AUTO_IO)
	message("Please patch with pull request https://github.com/PointCloudLibrary/pcl/pull/1294")
endif(NOT PCL_HAS_AUTO_IO)
unset(CMAKE_REQUIRED_FLAGS)
unset(CMAKE_REQUIRED_INCLUDES)

# Check for latest opencv
find_package(OpenCV 3 COMPONENTS core highgui imgproc xfeatures2d)
if(OpenCV_FOUND)
	include_directories(${OpenCV_INCLUDE_DIRS})
	add_executable (3DSelfie src/3DSelfie.cpp)
	target_link_libraries (3DSelfie ${PCL_LIBRARIES} ${OpenCV_LIBS})	
endif(OpenCV_FOUND)

#Add target only if pcl is patched
if(PCL_HAS_AUTO_IO)
	add_executable (ht_icp_check src/tools/ht_icp_check/ht_icp_check.cpp)
	target_link_libraries (ht_icp_check ${PCL_LIBRARIES} ${PCL_IO_LIBRARIES})
endif(PCL_HAS_AUTO_IO)

#Final Summary
message(" Modules that will be built: ")
message("")
if(OpenCV_FOUND)
	message("\t3DSelfie")
endif(OpenCV_FOUND)
if(PCL_HAS_AUTO_IO)
	message("\tht_icp_check")
endif(PCL_HAS_AUTO_IO)
message("")

message(" Modules that will NOT be built: ")
message("")
if(NOT OpenCV_FOUND)
	message("\t3DSelfie")
endif(NOT OpenCV_FOUND)
if(NOT PCL_HAS_AUTO_IO)
	message("\tht_icp_check")
endif(NOT PCL_HAS_AUTO_IO)