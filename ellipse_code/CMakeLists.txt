# requires minimum cmake version
cmake_minimum_required(VERSION 3.5)

# produce the cmake var PROJECT_NAME
project(shape_position)

if(NOT CMAKE_BUILD_TYPE)
set_property(CACHE CMAKE_BUILD_TYPE PROPERTY VALUE "Release")
endif()

#include(GNUInstallDirs)

# mandatory use of these packages
find_package(YCM REQUIRED)
find_package(OpenCV REQUIRED)
find_package(YARP COMPONENTS OS sig math dev idl_tools rosmsg REQUIRED)
find_package(event-driven REQUIRED)

add_executable(step step-by-step.cpp)
add_executable(${PROJECT_NAME} main.cpp eros.h affine.h)

target_link_libraries(${PROJECT_NAME} PRIVATE YARP::YARP_OS
        YARP::YARP_init
        YARP::YARP_sig
        YARP::YARP_dev
        YARP::YARP_math
        ev::event-driven
        ${OpenCV_LIBRARIES})

target_link_libraries(step PRIVATE YARP::YARP_OS
        YARP::YARP_init
        YARP::YARP_sig
        YARP::YARP_dev
        YARP::YARP_math
        ev::event-driven
        ${OpenCV_LIBRARIES})

install(TARGETS ${PROJECT_NAME} DESTINATION ${CMAKE_INSTALL_BINDIR})

