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

#include <fl/util/profiling.hpp>

#include <dbot_ros/tracker_node.h>

#include <dbot_ros/utils/ros_interface.hpp>

namespace dbot
{
template <typename Tracker>
TrackerNode<Tracker>::TrackerNode(
    const std::shared_ptr<Tracker>& tracker,
    const std::shared_ptr<dbot::CameraData>& camera_data,
    const std::shared_ptr<TrackerPublisher<State>>& publisher)
    : tracker_(tracker), publisher_(publisher),
      camera_data_(camera_data)
{
}

template <typename Tracker>
void TrackerNode<Tracker>::tracking_callback(
    const sensor_msgs::Image& ros_image)
{
    auto image = ri::Ros2EigenVector<typename Obsrv::Scalar>(
        ros_image, camera_data_->downsampling_factor());

    current_state_ = tracker_->track(image);

    publisher_->publish(current_state_, ros_image, camera_data_);
}



template <typename Tracker>
auto TrackerNode<Tracker>::current_state() const -> const State &
{
    return current_state_;
}
}
