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
 * \file tracker_node.hpp
 * \date Januray 2016
 * \author Jan Issac (jan.issac@gmail.com)
 */

#pragma once

#include <dbot_ros/tracker_node.h>

#include <dbot_ros/utils/ros_interface.hpp>

namespace dbot
{
template <typename Tracker>
TrackerNode<Tracker>::TrackerNode(
    const std::shared_ptr<Tracker>& tracker,
    const std::shared_ptr<TrackerPublisher<State>>& publisher)
    : tracker_(tracker), publisher_(publisher)
{
}

template <typename Tracker>
void TrackerNode<Tracker>::tracking_callback(const sensor_msgs::Image& ros_image)
{
    auto image = ri::Ros2EigenVector<typename Obsrv::Scalar>(
        ros_image, tracker_->camera_data()->downsampling_factor());

    auto state = tracker_->track(image);
    publisher_->publish(state, ros_image, tracker_->camera_data());
}
}
