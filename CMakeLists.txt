cmake_minimum_required(VERSION 3.10)
project(multi_camera_calibration)

find_package(PkgConfig)
pkg_check_modules(YAMLCPP REQUIRED yaml-cpp>=0.5)

find_package(yaml-cpp REQUIRED)

if ( NOT yaml-cpp_FOUND )
    message( FATAL_ERROR "Could NOT find yaml-cpp" )
else ()
    message(STATUS "***FOUND*** yaml-cpp in directory ${yaml-cpp_DIR}.")
endif ()

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  sensor_msgs
  image_transport
  rosbag # Needed for rosbag manipulation
  roslib # Needed for ros::package::getPath
)
find_package(OpenCV REQUIRED)

## System dependencies are found with CMake's conventions
find_package(Eigen3 REQUIRED)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated -Wdeprecated-declarations")
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
message ( STATUS " CMake C++ FLAGS ${CMAKE_CXX_FLAGS}")

## We tell then system we have a catkin_package, no dependencies going forward
catkin_package()

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  src
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
)

###############
## Libraries ##
###############
## Declare C++ libraries here
add_library(${PROJECT_NAME}_my_library src/camera_wrapper.cpp src/calibration.cpp src/camera.cpp)

###############
## Executable ##
###############
## Declare C++ executable
add_executable(${PROJECT_NAME} src/main.cpp src/camera_wrapper.cpp src/calibration.cpp src/camera.cpp)

## Add cmake target dependencies of the executable, enforces generating messages
## VERY VERY VERY HIGHLY unlikely to change
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${YAMLCPP_LIBRARIES} ${PROJECT_NAME}_my_library)

#############
## Testing ##
#############
## Add gtest based cpp test target and link libraries