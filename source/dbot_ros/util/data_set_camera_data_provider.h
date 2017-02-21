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
 * \file data_set_camera_data_provider.h
 * \author Jan Issc (jan.issac@gmail.com)
 * \date January 2016
 */

#pragma once

#include <Eigen/Dense>
#include <dbot/camera_data_provider.h>
#include <dbot_ros/util/tracking_dataset.h>
#include <ros/ros.h>
#include <string>

namespace dbot
{
class DataSetCameraDataProvider : public CameraDataProvider
{
public:
    /**
     * \brief Creates a RosCameraDataProvider
     * \param nh
     * 			ros::NodeHandle instance used to read topic data
     * \param camera_info_topic
     * 			Camera info topic name
     * \param downsampling_factor
     * 			Resolution downsampling factor
     * \param timeout
     * 			Timeout in seconds applied on topic requests
     */
    DataSetCameraDataProvider(const std::shared_ptr<TrackingDataset> data_set,
                              int downsampling_factor);

public:
    /**
     * \brief returns an obtained depth image as an Eigen matrix from depth
     *        image topic
     */
    Eigen::MatrixXd depth_image() const;

    /**
     * \brief returns an obtained depth image as an Eigen vector
     */
    Eigen::VectorXd depth_image_vector() const;

    /**
     * \brief Reads and returns the camera matrix from the camera info topic and
     * adapts its enties if the downsampling_factor is not equal 1.
     */
    Eigen::Matrix3d camera_matrix() const;

    /**
     * \brief Reads and returns the current camera frame obtained from the
     * camera info topic
     */
    std::string frame_id() const;

    /**
     * \brief Returns the downsampling_factor defined on construction
     */
    int downsampling_factor() const;

    /**
     * \brief Returns the resolution of the depth camera
     */
    CameraData::Resolution native_resolution() const;

private:
    std::shared_ptr<TrackingDataset> data_set_;
    CameraData::Resolution native_resolution_;
    int downsampling_factor_;
};
}
