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
 * \file object_tracker_publisher.cpp
 * \date January 2016
 * \author Jan Issac (jan.issac@gmail.com)
 */

#include <Eigen/Core>
#include <Eigen/Dense>
#include <dbot/pose/euler_vector.h>
#include <dbot/pose/free_floating_rigid_bodies_state.h>
#include <dbot/pose/pose_vector.h>
#include <dbot/pose/pose_velocity_vector.h>
#include <dbot_ros/object_tracker_publisher.h>
#include <dbot_ros/util/ros_interface.h>
#include <dbot_ros_msgs/ObjectState.h>
#include <visualization_msgs/Marker.h>

namespace dbot
{
ObjectStatePublisher::ObjectStatePublisher(
    const dbot::ObjectResourceIdentifier& ori,
    int object_color_red,
    int object_color_green,
    int object_color_blue)
    : node_handle_("~"),
      ori_(ori),
      object_color_red_(object_color_red),
      object_color_green_(object_color_green),
      object_color_blue_(object_color_blue)
{
    object_marker_publisher_ =
        node_handle_.advertise<visualization_msgs::Marker>("object_model", 0);
    object_state_publisher_ =
        node_handle_.advertise<dbot_ros_msgs::ObjectState>("object_state", 0);
}

void ObjectStatePublisher::publish(
    const std::vector<dbot_ros_msgs::ObjectState>& states)

{
    for (int i = 0; i < ori_.count_meshes(); i++)
    {
        ri::publish_marker(states[i].pose,
                           ori_.mesh_uri(i),
                           object_marker_publisher_,
                           i,
                           object_color_red_ / 255.,
                           object_color_green_ / 255.,
                           object_color_blue_ / 255.);


        dbot_ros_msgs::ObjectState object_state_message = states[i];
        object_state_message.name          = ori_.mesh_without_extension(i);
        object_state_message.ori.name      = ori_.mesh(i);
        object_state_message.ori.directory = ori_.directory();
        object_state_message.ori.package   = ori_.package();
        object_state_publisher_.publish(object_state_message);
    }
}
}
