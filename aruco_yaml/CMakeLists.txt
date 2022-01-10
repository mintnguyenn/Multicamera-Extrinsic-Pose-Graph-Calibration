cmake_minimum_required(VERSION 3.10)

project(yaml_reader)

find_package(PkgConfig)
pkg_check_modules(YAMLCPP REQUIRED yaml-cpp>=0.5)


#get_directory_property( _vars_before VARIABLES )
find_package( OpenCV 4.0 REQUIRED
    PATHS /)

if ( NOT OpenCV_FOUND )
    message( FATAL_ERROR "Could NOT find OpenCV version 4.0 or higher" )
else ()
    message(STATUS "***FOUND*** OpenCV in directory ${OpenCV_DIR}.")
endif ()

find_package( yaml-cpp REQUIRED )

if ( NOT yaml-cpp_FOUND )
    message( FATAL_ERROR "Could NOT find yaml-cpp" )
else ()
    message(STATUS "***FOUND*** yaml-cpp in directory ${yaml-cpp_DIR}.")
endif ()

add_executable(yaml_reader main.cpp)
target_link_libraries(yaml_reader ${OpenCV_LIBS} ${YAMLCPP_LIBRARIES})
