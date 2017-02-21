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

#include <dbot_ros/util/data_set_camera_data_provider.h>
#include <dbot_ros/util/ros_interface.h>
#include <fl/util/profiling.hpp>
#include <sensor_msgs/Image.h>

namespace dbot
{
DataSetCameraDataProvider::DataSetCameraDataProvider(
    const std::shared_ptr<TrackingDataset> data_set, int downsampling_factor)
    : data_set_(data_set), downsampling_factor_(downsampling_factor)
{
    auto ros_image            = data_set_->GetImage(0);
    native_resolution_.height = ros_image->height;
    native_resolution_.width  = ros_image->width;
}

Eigen::MatrixXd DataSetCameraDataProvider::depth_image() const
{
    auto ros_image = data_set_->GetImage(0);

    auto image = ri::to_eigen_matrix<double>(*ros_image, downsampling_factor_);

    return image;
}

Eigen::VectorXd DataSetCameraDataProvider::depth_image_vector() const
{
    auto ros_image = data_set_->GetImage(0);

    auto image = ri::to_eigen_vector<double>(*ros_image, downsampling_factor_);

    return image;
}

Eigen::Matrix3d DataSetCameraDataProvider::camera_matrix() const
{
    auto camera_matrix = data_set_->GetCameraMatrix(0);

    camera_matrix.topLeftCorner(2, 3) /= downsampling_factor_;

    return camera_matrix;
}

std::string DataSetCameraDataProvider::frame_id() const
{
    return data_set_->GetInfo(0)->header.frame_id;
}

int DataSetCameraDataProvider::downsampling_factor() const
{
    return downsampling_factor_;
}

CameraData::Resolution DataSetCameraDataProvider::native_resolution() const
{
    return native_resolution_;
}
}
