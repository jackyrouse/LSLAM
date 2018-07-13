cmake_minimum_required(VERSION 3.10)
project(LSLAM)

set(CMAKE_CXX_STANDARD 11)

list(APPEND testlaser_LIBRARIES ${CMAKE_CURRENT_SOURCE_DIR}/lib/libydlidar_driver.so)

find_package(cartographer REQUIRED)
find_package(Boost 1.55.0 REQUIRED COMPONENTS system filesystem)

include_directories(include/
        /usr/include/eigen3/
        ${Boost_INCLUDE_DIRS}
        ydlidar)

add_executable(LSLAM main.cpp
        include/cartographer_ros/node_constants.cc
        include/cartographer_ros/trajectory_options.cc
        include/cartographer_ros/node_options.cc
        include/cartographer_ros/ros_log_sink.cc
        include/cartographer_ros/time_conversion.cc
        include/cartographer_ros/msg_conversion.cc
        include/cartographer_ros/tf_bridge.cc
        include/cartographer_ros/map_builder_bridge.cc
        include/cartographer_ros/sensor_bridge.cc
        include/cartographer_ros/node.cc)


target_link_libraries(${PROJECT_NAME} PUBLIC cartographer ${testlaser_LIBRARIES} ${Boost_LIBRARIES} pthread)