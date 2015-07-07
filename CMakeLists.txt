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
    cv_bridge
    urdf
    orocos_kdl
    kdl_parser
    rosbag
    message_filters
    robot_state_pub
    image_transport
    fl
    dbot
)


set(PROJECT_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")
set(CMAKE_MODULE_PATH ${PROJECT_MODULE_PATH})

find_package(Boost REQUIRED)
find_package(Eigen REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL 1.3 REQUIRED)

set(ccache_DIR ${PROJECT_MODULE_PATH})
find_package(ccache REQUIRED)

if(CCACHE_FOUND)
    message(" ccache found. GCC will be launched via ccache.")
    set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE ccache)
    set_property(GLOBAL PROPERTY RULE_LAUNCH_LINK ccache)
else(CCACHE_FOUND)
    message(" ccache not found. ")
endif(CCACHE_FOUND)

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
        pose_tracking_interface
    CATKIN_DEPENDS
        roscpp
        roslib
        sensor_msgs
        urdf
        orocos_kdl
        kdl_parser
        message_filters
        robot_state_pub
        image_transport
        fl
        dbot
    DEPENDS
        OpenCV
        eigen
        assimp
)

###########
## Build ##
###########

include_directories(include ${catkin_INCLUDE_DIRS} )

set(PROJECT_NAME "dbot_ros_pkg")

file(GLOB_RECURSE headers include/${PROJECT_NAME}/*.hpp
                          include/${PROJECT_NAME}/*.h)

set(sources
    src/${PROJECT_NAME}/trackers/tracker.cpp
    src/${PROJECT_NAME}/utils/object_file_reader.cpp
    src/${PROJECT_NAME}/utils/pcl_interface.cpp
    src/${PROJECT_NAME}/utils/ros_interface.cpp
)


add_library(${PROJECT_NAME} ${dbot_headers}
                            ${headers}
                            ${sources})


add_executable(tracker  src/${PROJECT_NAME}/trackers/tracker_node.cpp)
target_link_libraries(tracker
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
    yaml-cpp)


target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES}
                                      ${catkin_LIBRARIES}
                                      ${OpenCV_LIBRARIES})






