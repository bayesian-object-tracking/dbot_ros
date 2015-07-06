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



find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

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



# enable cuda debug information with -g -G -O0, to use with cuda-dbg
# use --ptxas-options=-v to see number of registers, local, shared and constant memory used in kernels
set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} -O2 -arch=sm_20)



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










set(PROJECT_NAME_TRACKING_INTERFACE "dbot_ros_pkg")

file(GLOB_RECURSE tracking_interface_headers include/${PROJECT_NAME_TRACKING_INTERFACE}/*.hpp include/${PROJECT_NAME_TRACKING_INTERFACE}/*.h)

set(tracking_interface_sources
    src/${PROJECT_NAME_TRACKING_INTERFACE}/trackers/tracker.cpp

    src/${PROJECT_NAME_TRACKING_INTERFACE}/utils/image_visualizer.cpp
    src/${PROJECT_NAME_TRACKING_INTERFACE}/utils/cloud_visualizer.cpp
    src/${PROJECT_NAME_TRACKING_INTERFACE}/utils/object_file_reader.cpp
    src/${PROJECT_NAME_TRACKING_INTERFACE}/utils/tracking_dataset.cpp
    src/${PROJECT_NAME_TRACKING_INTERFACE}/utils/pcl_interface.cpp
    src/${PROJECT_NAME_TRACKING_INTERFACE}/utils/ros_interface.cpp
    src/${PROJECT_NAME_TRACKING_INTERFACE}/utils/image_publisher.cpp
)


add_library(${PROJECT_NAME_TRACKING_INTERFACE} ${dbot_headers}
                                               ${tracking_interface_headers}
                                               ${tracking_interface_sources})






    add_executable(tracker
        src/${PROJECT_NAME_TRACKING_INTERFACE}/trackers/tracker_node.cpp)
    target_link_libraries(tracker
        ${PROJECT_NAME_TRACKING_INTERFACE}
        ${catkin_LIBRARIES}
        ${OpenCV_LIBS}
        ${PCL_LIBRARIES}
        yaml-cpp)


    target_link_libraries(${PROJECT_NAME_TRACKING_INTERFACE} ${PCL_LIBRARIES}
                                                             ${catkin_LIBRARIES}
                                                             ${OpenCV_LIBRARIES}
                                                             assimp)






