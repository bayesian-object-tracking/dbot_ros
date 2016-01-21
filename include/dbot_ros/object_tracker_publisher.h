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

#include <ros/ros.h>

#include <sensor_msgs/Image.h>
#include <dbot/util/camera_data.hpp>
#include <dbot/util/object_model.hpp>
#include <dbot/util/object_resource_identifier.hpp>
#include <dbot_ros/tracker_publisher.h>

namespace dbot
{
/**
 * \brief Represents the object tracker publisher. This publishes the object
 * estimated state and its marker.
 */
template <typename Tracker>
class ObjectTrackerPublisher
        : public TrackerPublisher<Tracker>
{
public:
    typedef typename Tracker::State State;

public:
    ObjectTrackerPublisher(const dbot::ObjectResourceIdentifier& ori);

    void publish(const State& state,
                 const sensor_msgs::Image& image,
                 const std::shared_ptr<dbot::CameraData>& camera_data);

protected:
    ros::NodeHandle node_handle_;
    ros::Publisher object_marker_publisher_;
    ros::Publisher object_state_publisher_;
    dbot::ObjectResourceIdentifier ori_;
};
}
