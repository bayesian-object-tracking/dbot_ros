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

#include <Eigen/Dense>
#include <Eigen/Core>
#include <osr/euler_vector.hpp>
#include <osr/pose_velocity_vector.hpp>
#include <osr/pose_vector.hpp>
#include <osr/free_floating_rigid_bodies_state.hpp>
#include <dbot_ros/util/ros_interface.hpp>
#include <dbot_ros/object_tracker_publisher.h>

/* ros messages */
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

void ObjectStatePublisher::publish(const geometry_msgs::PoseStamped& pose)
{
    ri::publish_marker(pose,
                       ori_.mesh_uri(0),
                       object_marker_publisher_,
                       0,
                       object_color_red_ / 255.,
                       object_color_green_ / 255.,
                       object_color_blue_ / 255.);

    ri::publish_pose(pose,
                     ori_.mesh(0),
                     ori_.directory(),
                     ori_.package(),
                     object_state_publisher_);
}

void ObjectStatePublisher::publish(
    const std::vector<geometry_msgs::PoseStamped>& poses)
{
    for (int i = 0; i < ori_.count_meshes(); i++)
    {
        ri::publish_marker(poses[i],
                           ori_.mesh_uri(i),
                           object_marker_publisher_,
                           i,
                           object_color_red_ / 255.,
                           object_color_green_ / 255.,
                           object_color_blue_ / 255.);

        ri::publish_pose(poses[i],
                         ori_.mesh(i),
                         ori_.directory(),
                         ori_.package(),
                         object_state_publisher_);
    }
}

void ObjectStatePublisher::publish(
    const std::vector<dbot_ros_msgs::ObjectState>& states)

{
    for (int i = 0; i < ori_.count_meshes(); i++)
    {
        ri::publish_marker(states[i].pose,
                           ori_.mesh_uri(i),
                           object_marker_publisher_,
                           0,
                           object_color_red_ / 255.,
                           object_color_green_ / 255.,
                           object_color_blue_ / 255.);


        dbot_ros_msgs::ObjectState object_state_message = states[i];
        object_state_message.ori.name      = ori_.mesh_uri(i);
        object_state_message.ori.directory = ori_.directory();
        object_state_message.ori.package   = ori_.package();
        object_state_publisher_.publish(states[i]);
    }
}
}
