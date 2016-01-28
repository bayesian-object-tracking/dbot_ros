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
 * \file tracker_publisher.h
 * \date January 2016
 * \author Jan Issac (jan.issac@gmail.com)
 */

#pragma once

#include <sensor_msgs/Image.h>
#include <dbot/util/camera_data.hpp>

namespace dbot
{
/**
 * \brief Represents the tracker publisher interface. A tracker publisher may
 * publish the estimated state result along object markers, robot models,
 * point clouds and images.
 */
template <typename State>
class TrackerPublisher
{
public:
    virtual void publish(
        State& state,
        const sensor_msgs::Image& image,
        const std::shared_ptr<dbot::CameraData>& camera_data) = 0;
};
}
