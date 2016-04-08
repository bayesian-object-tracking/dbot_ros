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

find_package(catkin REQUIRED
  roscpp
  roslib
  sensor_msgs
  std_msgs
  geometry_msgs
  cv_bridge
  rosbag
  message_generation
  image_transport
  fl
  opi
  dbot
  dbot_ros_msgs
  )

#Generate messages in the 'msg' folder
add_message_files(
  FILES
    ObjectState.msg)

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
    std_msgs
    geometry_msgs
    dbot_ros_msgs)

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
    include
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    roscpp
    roslib
    sensor_msgs
    std_msgs
    geometry_msgs
    message_runtime
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

include_directories(include ${catkin_INCLUDE_DIRS} )

file(GLOB_RECURSE headers include/${PROJECT_NAME}/*.hpp
              include/${PROJECT_NAME}/*.h)

set(sources
  ${headers}
  src/${PROJECT_NAME}/object_tracker_publisher.cpp
  src/${PROJECT_NAME}/object_tracker_node.cpp
  src/${PROJECT_NAME}/utils/ros_camera_data_provider.cpp
  src/${PROJECT_NAME}/utils/data_set_camera_data_provider.cpp
#  src/${PROJECT_NAME}/utils/pcl_interface.cpp
  src/${PROJECT_NAME}/utils/ros_interface.cpp
#  src/${PROJECT_NAME}/utils/cloud_visualizer.cpp
  src/${PROJECT_NAME}/utils/tracking_dataset.cpp
)

add_library(${PROJECT_NAME}
  ${dbot_headers}
  ${headers}
  ${sources})

add_dependencies(${PROJECT_NAME}
  dbot_ros_generate_messages_cpp)

target_link_libraries(${PROJECT_NAME}
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES})

add_executable(rbc_particle_filter_object_tracker
  src/${PROJECT_NAME}/trackers/rbc_particle_filter_object_tracker_node.cpp)

add_dependencies(rbc_particle_filter_object_tracker
  dbot_ros_generate_messages_cpp)

target_link_libraries(rbc_particle_filter_object_tracker
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
  yaml-cpp)

add_executable(object_tracker_service
  src/${PROJECT_NAME}/trackers/object_tracker_service_node.cpp)

add_dependencies(object_tracker_service
  dbot_ros_generate_messages_cpp)

target_link_libraries(object_tracker_service
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
  yaml-cpp)

add_executable(rms_gaussian_filter_object_tracker
  src/${PROJECT_NAME}/trackers/rms_gaussian_filter_object_tracker_node.cpp)

add_dependencies(rms_gaussian_filter_object_tracker
  dbot_ros_generate_messages_cpp)

target_link_libraries(rms_gaussian_filter_object_tracker
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
  yaml-cpp)
