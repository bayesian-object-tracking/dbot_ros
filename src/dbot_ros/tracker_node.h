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
 * \file ros_object_tracker.h
 * \date Januray 2016
 * \author Jan Issac (jan.issac@gmail.com)
 */

#pragma once

#include <vector>
#include <memory>
#include <mutex>
#include <dbot/common/camera_data.hpp>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PoseStamped.h>

namespace dbot
{
/**
 * \brief Represents a generic tracker node
 */
template <typename Tracker>
class ObjectTrackerRos
{
public:
    typedef typename Tracker::State State;
    typedef typename Tracker::Obsrv Obsrv;

public:
    /**
     * \brief Creates a ObjectTrackerRos
     */
    ObjectTrackerRos(const std::shared_ptr<Tracker>& tracker,
                const std::shared_ptr<dbot::CameraData>& camera_data);

    /**
     * \brief Tracking callback function which is invoked whenever a new image
     *        is available
     */
    void track(const sensor_msgs::Image& ros_image);

    void initialize(const std::vector<State>& initial_states);

    /**
     * \brief Incoming observation callback function
     * \param ros_image new observation
     */
    void update_obsrv(const sensor_msgs::Image& ros_image);

    void run();
    bool run_once();

    const State& current_state() const;
    const geometry_msgs::PoseStamped& current_pose() const;

    const std::shared_ptr<Tracker>& tracker() { return tracker_; }

    void shutdown();

protected:
    bool obsrv_updated_;
    bool running_;
    State current_state_;
    geometry_msgs::PoseStamped current_pose_;
    sensor_msgs::Image current_ros_image_;
    std::mutex obsrv_mutex_;
    std::shared_ptr<Tracker> tracker_;
    std::shared_ptr<dbot::CameraData> camera_data_;
};


}
