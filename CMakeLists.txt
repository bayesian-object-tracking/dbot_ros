cmake_minimum_required(VERSION 2.8.3)
project(dbot_ros)

############################
# Options                  #
############################

############################
# Flags                    #
############################
# Enable c++11 GCC 4.6 or greater required
add_definitions(-std=c++0x)
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
set(dbot_ros_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/src)

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
  opi
  dbot
  dbot_ros_msgs
  )

set(PROJECT_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")
set(CMAKE_MODULE_PATH ${PROJECT_MODULE_PATH})

if(NOT dbot_FOUND)
  find_package(dbot REQUIRED)
endif(NOT dbot_FOUND)

if(NOT osr_FOUND)
  find_package(osr REQUIRED)
endif(NOT osr_FOUND)

find_package(Boost REQUIRED)
find_package(Eigen REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(${dbot_INCLUDE_DIRS})
include_directories(${osr_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIRS})
include_directories(${Eigen_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS
    src
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
    opi
    osr
    dbot
    dbot_ros_msgs
  DEPENDS
    OpenCV
    eigen)

###########
## Build ##
###########

include_directories(${dbot_ros_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

set(dbot_ros_SOURCE_DIR src/${PROJECT_NAME})

file(GLOB_RECURSE dbot_ros_HEADERS
    ${dbot_SOURCE_DIR}/*.hpp
    ${dbot_SOURCE_DIR}/*.h)

set(dbot_ros_SOURCES
    src/${PROJECT_NAME}/object_tracker_ros.cpp
    src/${PROJECT_NAME}/object_tracker_publisher.cpp 
    src/${PROJECT_NAME}/util/ros_camera_data_provider.cpp
    src/${PROJECT_NAME}/util/data_set_camera_data_provider.cpp
    src/${PROJECT_NAME}/util/ros_interface.cpp
    src/${PROJECT_NAME}/util/tracking_dataset.cpp)

add_library(${PROJECT_NAME}
  ${dbot_ros_HEADERS}
  ${dbot_ros_SOURCES})

add_dependencies(${PROJECT_NAME}
  dbot_ros_generate_messages_cpp)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES})

add_executable(rbc_particle_filter_object_tracker
  src/${PROJECT_NAME}/tracker/rbc_particle_filter_object_tracker_node.cpp)
add_dependencies(rbc_particle_filter_object_tracker
  dbot_ros_generate_messages_cpp)
target_link_libraries(rbc_particle_filter_object_tracker
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
  yaml-cpp)

add_executable(rms_gaussian_filter_object_tracker
  src/${PROJECT_NAME}/tracker/rms_gaussian_filter_object_tracker_node.cpp)
add_dependencies(rms_gaussian_filter_object_tracker
  dbot_ros_generate_messages_cpp)
target_link_libraries(rms_gaussian_filter_object_tracker
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
  yaml-cpp)

add_executable(object_tracker_service
  src/${PROJECT_NAME}/tracker/object_tracker_service_node.cpp)
add_dependencies(object_tracker_service
  dbot_ros_generate_messages_cpp)
target_link_libraries(object_tracker_service
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
  yaml-cpp)

add_executable(object_tracker_controller_service
  src/${PROJECT_NAME}/object_tracker_controller_service_node.cpp)
add_dependencies(object_tracker_controller_service
  dbot_ros_generate_messages_cpp)
target_link_libraries(object_tracker_controller_service
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
  yaml-cpp)

