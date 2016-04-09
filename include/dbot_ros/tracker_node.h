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
 * \file tracker_node.h
 * \date Januray 2016
 * \author Jan Issac (jan.issac@gmail.com)
 */

#pragma once

#include <memory>
#include <mutex>
#include <dbot_ros/tracker_publisher.h>

namespace dbot
{
/**
 * \brief Represents a generic tracker node
 */
template <typename Tracker>
class TrackerNode
{
public:
    typedef typename Tracker::State State;
    typedef typename Tracker::Obsrv Obsrv;

public:
    /**
     * \brief Creates a TrackerNode
     */
    TrackerNode(const std::shared_ptr<Tracker>& tracker,
                const std::shared_ptr<dbot::CameraData>& camera_data,
                const std::shared_ptr<TrackerPublisher<State>>& publisher);

    /**
     * \brief Tracking callback function which is invoked whenever a new image
     *        is available
     */
    void tracking_callback(const sensor_msgs::Image& ros_image);

    /**
     * \brief Incoming observation callback function
     * \param ros_image new observation
     */
    void obsrv_callback(const sensor_msgs::Image& ros_image);

//    void run();
    void run_once();

    const State& current_state() const;

    std::shared_ptr<Tracker> tracker() { return tracker_; }
    std::shared_ptr<dbot::CameraData>

    camera_data() const
    {
        return camera_data_;
    }

    void shutdown();

protected:
    bool obsrv_updated_;
    bool running_;
    State current_state_;
    sensor_msgs::Image current_ros_image_;
    std::mutex obsrv_mutex_;
    std::shared_ptr<Tracker> tracker_;
    std::shared_ptr<dbot::CameraData> camera_data_;
    std::shared_ptr<TrackerPublisher<State>> publisher_;
};


}
