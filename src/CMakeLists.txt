cmake_minimum_required(VERSION 2.8)

SET(CMAKE_MODULE_PATH ${CMAKE_INSTALL_PREFIX}/lib/cmake/ )
SET(CMAKE_CXX_FLAGS "-std=c++0x")

find_package(aruco REQUIRED )
# Find OpenCV, you may need to set OpenCV_DIR variable
# to the absolute path to the directory containing OpenCVConfig.cmake file
# via the command line or GUI
find_package(OpenCV REQUIRED)

# If the package has been found, several variables will
# be set, you can find the full list with descriptions
# in the OpenCVConfig.cmake file.
# Print some message showing some of them
message(STATUS "OpenCV library status:")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")

# Add OpenCV headers location to your include paths
#MESSAGE(STATUS "Head files Directory: ${PROJECT_SOURCE_DIR}/include")
#declaration the include files directories
include_directories(${OpenCV_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/include)

#Add Eigen3 headers location to your include paths
SET( EIGEN3_INCLUDE_DIR "$ENV{EIGEN3_INCLUDE_DIR}" )
IF( NOT EIGEN3_INCLUDE_DIR )
    MESSAGE( FATAL_ERROR "Please point the environment variable EIGEN3_INCLUDE_DIR to the include directory of your Eigen3 installation.")
ENDIF()
include_directories ( ${EIGEN3_INCLUDE_DIR}s )
message(STATUS "    include path: ${EIGEN3_INCLUDE_DIR}")

#MESSAGE(STATUS "Lib Directory: ${PROJECT_SOURCE_DIR}/lib")
# declaration the lib directories
link_directories(${PROJET_SOURCE_DIR}/lib)

include_directories(${PROJET_SOURCE_DIR}/include)

add_executable(aruco_simple aruco_simple.cpp)
target_link_libraries(aruco_simple ${aruco_LIBS} ${OpenCV_LIBS})
