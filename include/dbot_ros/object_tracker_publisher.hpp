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
 * \file object_tracker_publisher.hpp
 * \date January 2016
 * \author Jan Issac (jan.issac@gmail.com)
 */

#pragma once

#include <Eigen/Dense>
#include <Eigen/Core>
#include <osr/euler_vector.hpp>
#include <osr/pose_velocity_vector.hpp>
#include <osr/pose_vector.hpp>
#include <osr/free_floating_rigid_bodies_state.hpp>
#include <dbot_ros/utils/ros_interface.hpp>
#include <dbot_ros/object_tracker_publisher.h>

/* ros messages */
#include <dbot_ros/ObjectState.h>
#include <visualization_msgs/Marker.h>

namespace dbot
{
template <typename Tracker>
ObjectTrackerPublisher<Tracker>::ObjectTrackerPublisher(
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
        node_handle_.advertise<dbot_ros::ObjectState>("object_state", 0);
}

template <typename Tracker>
void ObjectTrackerPublisher<Tracker>::publish(
    const State& state,
    const sensor_msgs::Image& image,
    const std::shared_ptr<dbot::CameraData>& camera_data)
{
    std_msgs::Header header = image.header;
    for (int i = 0; i < ori_.count_meshes(); i++)
    {
        ri::PublishMarker(
            state.component(i).homogeneous().template cast<float>(),
            header,
            ori_.mesh_uri(i),
            object_marker_publisher_,
            i,
            object_color_red_ / 255.,
            object_color_green_ / 255.,
            object_color_blue_ / 255.);

        ri::PublishObjectState(
            state.component(i).homogeneous().template cast<float>(),
            header,
            ori_.mesh_without_extension(i),
            object_state_publisher_);
    }
}
}
