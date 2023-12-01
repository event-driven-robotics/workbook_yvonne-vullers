# requires minimum cmake version
cmake_minimum_required(VERSION 3.5)

# produce the cmake var PROJECT_NAME
project(ellipse)

find_package(YCM REQUIRED)
find_package(OpenCV REQUIRED)
find_package(YARP COMPONENTS os sig REQUIRED)
find_package(event-driven REQUIRED)

add_executable(${PROJECT_NAME} create_ellipse.cpp)
add_executable(data readData.cpp)

target_link_libraries(${PROJECT_NAME} PRIVATE YARP::YARP_os
        YARP::YARP_init
        YARP::YARP_sig
        ev::event-driven
        ${OpenCV_LIBRARIES})

target_include_directories(${PROJECT_NAME} PRIVATE ${PROJECT_SOURCE_DIR}/include)

install(TARGETS ${PROJECT_NAME} DESTINATION ${CMAKE_INSTALL_BINDIR})
