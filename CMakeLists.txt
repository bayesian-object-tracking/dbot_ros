cmake_minimum_required(VERSION 2.8.3)
project(dbot_ros_pkg)

set(CMAKE_BUILD_TYPE Release)
#set(CMAKE_BUILD_TYPE Debug)
add_definitions(-std=c++0x -O3 -g3)
add_definitions(-DPROFILING_ON=1) #print profiling output

find_package(catkin REQUIRED
    roscpp
    roslib
    sensor_msgs
    cv_bridge
    rosbag
    message_filters
    robot_state_pub
    image_transport
    fl
    dbot
)

# find_package(Doxygen)
#
# if(DOXYGEN_FOUND)
#     configure_file(${CMAKE_CURRENT_SOURCE_DIR}/doxygen/Doxyfile.in ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile @ONLY)
#     add_custom_target(doc_dbot_ros_pkg
#         ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile
#         WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
#         COMMENT "Generating API documentation with Doxygen" VERBATIM)
# endif(DOXYGEN_FOUND)

set(PROJECT_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")
set(CMAKE_MODULE_PATH ${PROJECT_MODULE_PATH})

find_package(Boost REQUIRED)
find_package(Eigen REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL 1.3 REQUIRED)

set(ccache_DIR ${PROJECT_MODULE_PATH})
find_package(ccache)

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
        dbot_ros_pkg
    CATKIN_DEPENDS
        roscpp
        roslib
        sensor_msgs
        message_filters
        robot_state_pub
        image_transport
        fl
        dbot
    DEPENDS
        OpenCV
        eigen
)

###########
## Build ##
###########

include_directories(include ${catkin_INCLUDE_DIRS} )

set(PROJECT_NAME "dbot_ros_pkg")

file(GLOB_RECURSE
     library_headers
     include/${PROJECT_NAME}/*.hpp
     include/${PROJECT_NAME}/*.h)

set(library_sources
    src/${PROJECT_NAME}/utils/image_visualizer.cpp
    src/${PROJECT_NAME}/utils/cloud_visualizer.cpp
    src/${PROJECT_NAME}/utils/object_file_reader.cpp
    src/${PROJECT_NAME}/utils/tracking_dataset.cpp
    src/${PROJECT_NAME}/utils/robot_tracking_dataset.cpp
    src/${PROJECT_NAME}/utils/pcl_interface.cpp
    src/${PROJECT_NAME}/utils/ros_interface.cpp
    src/${PROJECT_NAME}/utils/image_publisher.cpp
)

add_library(${PROJECT_NAME} ${library_headers} ${library_sources})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

## dev_test_tracker
add_executable(dev_test_tracker
    src/${PROJECT_NAME}/trackers/dev_test_tracker_node.cpp
    src/${PROJECT_NAME}/trackers/dev_test_tracker/virtual_object.hpp
    src/${PROJECT_NAME}/trackers/dev_test_tracker/vector_hashing.hpp
)

target_link_libraries(dev_test_tracker
    ${PROJECT_NAME_TRACKING}
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
    ${CUDA_LIBRARIES}
    yaml-cpp)


#    ## fpgf_test_tracker
#    add_executable(fpgf_test_tracker
#        src/${PROJECT_NAME}/trackers/fpgf_test_tracker_node.cpp
#        src/${PROJECT_NAME}/trackers/dev_test_tracker/virtual_object.hpp
#        src/${PROJECT_NAME}/trackers/dev_test_tracker/vector_hashing.hpp
#    )

#    target_link_libraries(fpgf_test_tracker
#        ${PROJECT_NAME_TRACKING}
#        ${PROJECT_NAME}
#        ${catkin_LIBRARIES}
#        ${OpenCV_LIBS}
#        ${PCL_LIBRARIES}
#        ${CUDA_LIBRARIES}
#        yaml-cpp)

#    ## test_param_estimation_ukf
#    add_executable(test_param_estimation_ukf
#        src/${PROJECT_NAME}/trackers/test_param_estimation_ukf.hpp
#        src/${PROJECT_NAME}/trackers/test_param_estimation_ukf_node.cpp
#    )

#    target_link_libraries(test_param_estimation_ukf
#        ${PROJECT_NAME_TRACKING}
#        ${PROJECT_NAME}
#        ${PROJECT_NAME_TRACKING}_gpu
#        ${PROJECT_NAME_TRACKING}_cuda
#        ${catkin_LIBRARIES}
#        ${OpenCV_LIBS}
#        ${PCL_LIBRARIES}
#        ${CUDA_LIBRARIES}
#        yaml-cpp)

## test_param_estimation_ukf
add_executable(test_param_estimation_fpf
    src/${PROJECT_NAME}/fpf_test/depth_pixel_model.hpp
    src/${PROJECT_NAME}/fpf_test/squared_feature_policy.hpp
    src/${PROJECT_NAME}/fpf_test/test_param_estimation_fpf.hpp
    src/${PROJECT_NAME}/fpf_test/test_param_estimation_fpf_node.cpp
)

target_link_libraries(test_param_estimation_fpf
    ${PROJECT_NAME_TRACKING}
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
    ${CUDA_LIBRARIES}
    yaml-cpp)


### object_tracker
#add_executable(object_tracker
    #src/${PROJECT_NAME}/trackers/object_tracker_node.cpp)
#target_link_libraries(object_tracker
    #${PROJECT_NAME}
    #${catkin_LIBRARIES}
    #${OpenCV_LIBS}
    #${PCL_LIBRARIES}
    #yaml-cpp)

### robot_tracker
#add_executable(robot_tracker
    #src/${PROJECT_NAME}/trackers/robot_tracker_node.cpp)
#target_link_libraries(robot_tracker
    #${PROJECT_NAME}
    #${catkin_LIBRARIES}
    #${OpenCV_LIBS}
    #${PCL_LIBRARIES}
    #yaml-cpp)


### robot_tracker
#add_executable(robot_tracker_evaluation
    #src/${PROJECT_NAME}/trackers/robot_tracker_evaluation_node.cpp)
#target_link_libraries(robot_tracker_evaluation
    #${PROJECT_NAME}
    #${catkin_LIBRARIES}
    #${OpenCV_LIBS}
    #${PCL_LIBRARIES}
    #yaml-cpp)

###########
## Tests ##
###########
