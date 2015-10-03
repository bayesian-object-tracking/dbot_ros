cmake_minimum_required(VERSION 2.8.3)
project(dbot_ros_pkg)

set(CMAKE_BUILD_TYPE Release)
#set(CMAKE_BUILD_TYPE Debug)
add_definitions(-std=c++0x )
add_definitions(-DPROFILING_ON=1) #print profiling output

find_package(catkin REQUIRED
    roscpp
    roslib
    sensor_msgs
    std_msgs
    geometry_msgs
    cv_bridge
    urdf
    orocos_kdl
    kdl_parser
    rosbag
    message_generation
    image_transport
    fl
    dbot
    interactive_markers
)

#Generate messages in the 'msg' folder
add_message_files(
  FILES
  ObjectState.msg
)

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  )

set(PROJECT_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")
set(CMAKE_MODULE_PATH ${PROJECT_MODULE_PATH})

find_package(Boost REQUIRED)
find_package(Eigen REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL 1.3 REQUIRED)

include_directories(${Boost_INCLUDE_DIRS})
include_directories(${Eigen_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
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
        urdf
        orocos_kdl
        kdl_parser
        message_runtime
        message_filters
        image_transport
        fl
        dbot
        interactive_markers
    DEPENDS
        OpenCV
        eigen
)

###########
## Build ##
###########

include_directories(include ${catkin_INCLUDE_DIRS} )

set(PROJECT_NAME "dbot_ros_pkg")

file(GLOB_RECURSE headers include/${PROJECT_NAME}/*.hpp
                          include/${PROJECT_NAME}/*.h)

set(sources
#    src/${PROJECT_NAME}/trackers/tracker.cpp
    src/${PROJECT_NAME}/trackers/object_tracker.cpp
    src/${PROJECT_NAME}/utils/object_file_reader.cpp
    src/${PROJECT_NAME}/utils/pcl_interface.cpp
    src/${PROJECT_NAME}/utils/ros_interface.cpp
    src/${PROJECT_NAME}/utils/cloud_visualizer.cpp
    src/${PROJECT_NAME}/utils/tracking_dataset.cpp
)

add_library(${PROJECT_NAME} ${dbot_headers}
                            ${headers}
                            ${sources})

target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES}
                                      ${catkin_LIBRARIES}
                                      ${OpenCV_LIBRARIES})


# object_tracker
add_executable(object_tracker
    src/${PROJECT_NAME}/trackers/object_tracker_node.cpp)
target_link_libraries(object_tracker
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
    yaml-cpp)

#add_executable(tracker  src/${PROJECT_NAME}/trackers/tracker_node.cpp)
#target_link_libraries(tracker
#    ${PROJECT_NAME}
#    ${catkin_LIBRARIES}
#    ${OpenCV_LIBS}
#    ${PCL_LIBRARIES}
#    yaml-cpp)


#add_executable(interactive_tracker  src/${PROJECT_NAME}/trackers/tracker_interactive_node.cpp)
#target_link_libraries(interactive_tracker
#    ${PROJECT_NAME}
#    ${catkin_LIBRARIES}
#    ${OpenCV_LIBS}
#    ${PCL_LIBRARIES}
#    yaml-cpp)








