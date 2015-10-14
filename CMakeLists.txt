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

# Dependencies for 3DSelfie
#OpenCV
find_package(OpenCV 3 COMPONENTS core highgui imgproc xfeatures2d)
if(NOT OpenCV_FOUND)
	message("OpenCV3 not found. Disabling 3DSelfie.")
endif(NOT OpenCV_FOUND)
#OpenNI 
if(NOT OPENNI_FOUND)
	message("OpenNI not found. Disabling 3DSelfie.")
endif(NOT OPENNI_FOUND)

if(OpenCV_FOUND AND OPENNI_FOUND)
	include_directories(${OpenCV_INCLUDE_DIRS})
	add_executable (3DSelfie src/3DSelfie.cpp)
	target_link_libraries (3DSelfie ${PCL_LIBRARIES} ${OpenCV_LIBS})	
endif(OpenCV_FOUND AND OPENNI_FOUND)

# Dependencies for ht_icp_check
# Infamous pull request https://github.com/PointCloudLibrary/pcl/pull/1294
set(CMAKE_REQUIRED_FLAGS ${Boost_SYSTEM_LIBRARY_DEBUG})
set(CMAKE_REQUIRED_INCLUDES ${PCL_INCLUDE_DIRS} ${EIGEN_INCLUDE_DIRS})
CHECK_INCLUDE_FILE_CXX(pcl/io/auto_io.h PCL_HAS_AUTO_IO)
if(NOT PCL_HAS_AUTO_IO)
	message("Disabling 3DSelfie. Please pull pcl from https://github.com/SergioRAgostinho/pcl")
endif(NOT PCL_HAS_AUTO_IO)
unset(CMAKE_REQUIRED_FLAGS)
unset(CMAKE_REQUIRED_INCLUDES)
# Boost program_options
find_package(Boost 1.40 COMPONENTS program_options REQUIRED)
#final test
if(PCL_HAS_AUTO_IO AND Boost_FOUND)
	include_directories(${Boost_INCLUDE_DIRS})
	add_executable (ht_icp_check src/tools/ht_icp_check/ht_icp_check.cpp)
	target_link_libraries (ht_icp_check ${PCL_LIBRARIES} ${PCL_IO_LIBRARIES} ${Boost_LIBRARIES})
endif(PCL_HAS_AUTO_IO AND Boost_FOUND)

#Final Summary
message("\nModules that will be built:\n")

if(OpenCV_FOUND AND OPENNI_FOUND)
	message("\t- 3DSelfie")
endif(OpenCV_FOUND AND OPENNI_FOUND)
if(PCL_HAS_AUTO_IO AND Boost_FOUND)
	message("\t- ht_icp_check")
endif(PCL_HAS_AUTO_IO AND Boost_FOUND)

message("\nModules that will NOT be built:\n")
if(NOT (OpenCV_FOUND AND OPENNI_FOUND))
	message("\t- 3DSelfie")
endif(NOT (OpenCV_FOUND AND OPENNI_FOUND))
if(NOT (PCL_HAS_AUTO_IO AND Boost_FOUND))
	message("\t- ht_icp_check")
endif(NOT (PCL_HAS_AUTO_IO AND Boost_FOUND))