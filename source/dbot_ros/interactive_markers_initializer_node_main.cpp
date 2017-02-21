/*
 * This is part of the Bayesian Object Tracking (bot),
 * (https://github.com/bayesian-object-tracking)
 *
 * Copyright (c) 2015 Max Planck Society,
 * 				 Autonomous Motion Department,
 * 			     Institute for Intelligent Systems
 *
 * This Source Code Form is subject to the terms of the GNU General Public
 * License License (GNU GPL). A copy of the license can be found in the LICENSE
 * file distributed with this source code.
 */

/**
 * \file interactive_markers_initializer_node.cpp
 * \date October 2015
 * \author Jan Issac (jan.issac@gmail.com)
 */

#include <boost/filesystem.hpp>
#include <dbot_ros/util/interactive_marker_initializer.h>
#include <functional>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <ros/package.h>
#include <ros/publisher.h>
#include <string>

static ros::Publisher pose_publisher;

/**
 * \brief Callback function which is called when the marker object poses
 * have been updated
 */
void poses_updated_callback(const geometry_msgs::PoseArray& pose_array)
{
    pose_publisher.publish(pose_array);
}

int main(int argc, char** argv)
{
    /* ------------------------------ */
    /* - Setup ros                  - */
    /* ------------------------------ */
    ros::init(argc, argv, "interactive_markers_initializer_node");

    ros::NodeHandle nh("~");

    std::string camera_frame_id;
    std::string topic_name;
    std::string object_package;
    std::string object_directory;
    std::vector<std::string> object_meshes;

    /// \todo nh.getParam does not check whether the parameter exists in the
    /// config file. this is dangerous, we should use ri::read instead

    nh.getParam("topic_name", topic_name);
    nh.getParam("object/meshes", object_meshes);
    nh.getParam("object/package", object_package);
    nh.getParam("object/directory", object_directory);
    nh.getParam("camera_frame_id", camera_frame_id);

    opi::InteractiveMarkerInitializer im_server(camera_frame_id,
                                                object_package,
                                                object_directory,
                                                object_meshes,
                                                {},
                                                true);

    pose_publisher = nh.advertise<geometry_msgs::PoseArray>(topic_name, 1);

    im_server.poses_update_callback(
        std::bind(&poses_updated_callback, std::placeholders::_1));

    ROS_INFO("Object pose initializer node up and running...");
    ros::spin();

    return 0;
}
