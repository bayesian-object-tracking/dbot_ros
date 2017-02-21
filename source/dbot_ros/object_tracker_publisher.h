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
 * \file object_tracker_publisher.h
 * \date January 2016
 * \author Jan Issac (jan.issac@gmail.com)
 */

#pragma once

#include <dbot/camera_data.h>
#include <dbot/object_model.h>
#include <dbot/object_resource_identifier.h>
#include <dbot_ros_msgs/ObjectState.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <vector>

namespace dbot
{
/**
 * \brief Represents the object tracker publisher. This publishes the object
 * estimated state and its marker.
 */
class ObjectStatePublisher
{
public:
    ObjectStatePublisher(const dbot::ObjectResourceIdentifier& ori,
                         int object_color_red,
                         int object_color_green,
                         int object_color_blue);

    void publish(const std::vector<dbot_ros_msgs::ObjectState>& state);

protected:
    ros::NodeHandle node_handle_;
    ros::Publisher object_marker_publisher_;
    ros::Publisher object_state_publisher_;
    dbot::ObjectResourceIdentifier ori_;
    int object_color_red_;
    int object_color_green_;
    int object_color_blue_;
};
}
