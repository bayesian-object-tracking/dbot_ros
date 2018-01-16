cmake_minimum_required(VERSION 2.8.3)
project(dbot_ros)

############################
# Flags                    #
############################
# Enable c++11 GCC 4.6 or greater required
add_definitions(-std=c++11)
add_definitions(-DPROFILING_ON=1 -fPIC) #print profiling output

############################
# Library Version          #
############################
#include(cmake/version.cmake)

############################
# Library info summary     #
############################
#include(cmake/info.cmake)

#info_begin()
#  info_project("::bot::dbot_ros:: dbot Tracker ROS Pkg" ${PROJECT_VERSION})
#info_end()

############################
# Setup                    #
############################
set(CMAKE_EXPORT_COMPILE_COMMANDS 1)
set(dbot_ros_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/source)

############################
# Dependencies             #
############################
find_package(catkin REQUIRED
  roscpp
  roslib
  sensor_msgs
  std_msgs
  geometry_msgs
  cv_bridge
  rosbag
  image_transport
  fl
  dbot
  dbot_ros_msgs
  interactive_markers
  )

set(PROJECT_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")
set(CMAKE_MODULE_PATH ${PROJECT_MODULE_PATH})

find_package(Boost REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})

find_package(Eigen3 3.2 EXACT REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# find_package(OpenCV REQUIRED)
# include_directories(${OpenCV_INCLUDE_DIRS})

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS
    source
    ${dbot_ros_INCLUDE_DIRS}
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    roscpp
    roslib
    sensor_msgs
    std_msgs
    geometry_msgs
    message_filters
    image_transport
    fl
    dbot
    dbot_ros_msgs
    interactive_markers
    )

###########
## Build ##
###########

include_directories(${dbot_ros_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

set(dbot_ros_SOURCE_DIR source/${PROJECT_NAME})

file(GLOB_RECURSE dbot_ros_HEADERS
    ${dbot_SOURCE_DIR}/*.hpp
    ${dbot_SOURCE_DIR}/*.h)

set(dbot_ros_SOURCES
    source/${PROJECT_NAME}/object_tracker_ros.cpp
    source/${PROJECT_NAME}/object_tracker_publisher.cpp 
    source/${PROJECT_NAME}/util/ros_camera_data_provider.cpp
    source/${PROJECT_NAME}/util/data_set_camera_data_provider.cpp
    source/${PROJECT_NAME}/util/ros_interface.cpp
    source/${PROJECT_NAME}/util/tracking_dataset.cpp
    source/${PROJECT_NAME}/util/interactive_marker_initializer.cpp)

add_library(${PROJECT_NAME}
  ${dbot_ros_HEADERS}
  ${dbot_ros_SOURCES})

add_dependencies(${PROJECT_NAME}
  dbot_ros_msgs_generate_messages_cpp)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  )

add_executable(particle_tracker
  source/${PROJECT_NAME}/tracker/particle_tracker_node.cpp)
add_dependencies(particle_tracker
  dbot_ros_msgs_generate_messages_cpp)
target_link_libraries(particle_tracker
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  yaml-cpp)

add_executable(gaussian_tracker
  source/${PROJECT_NAME}/tracker/gaussian_tracker_node.cpp)
add_dependencies(gaussian_tracker
  dbot_ros_msgs_generate_messages_cpp)
target_link_libraries(gaussian_tracker
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  yaml-cpp)

add_executable(object_tracker_service
  source/${PROJECT_NAME}/tracker/object_tracker_service_node.cpp)
add_dependencies(object_tracker_service
  dbot_ros_msgs_generate_messages_cpp)
target_link_libraries(object_tracker_service
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  yaml-cpp)

add_executable(object_tracker_controller_service
  source/${PROJECT_NAME}/object_tracker_controller_service_node.cpp)
add_dependencies(object_tracker_controller_service
  dbot_ros_msgs_generate_messages_cpp)
target_link_libraries(object_tracker_controller_service
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  yaml-cpp)

add_executable(
  interactive_markers_initializer_node
  source/${PROJECT_NAME}/interactive_markers_initializer_node_main.cpp)
target_link_libraries(
  interactive_markers_initializer_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  )

