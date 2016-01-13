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
 * \file ros_camera_data_provider.hpp
 * \author Jan Issc (jan.issac@gmail.com)
 * \date December 2015
 */

#include <sensor_msgs/Image.h>

#include <dbot_ros/utils/ros_interface.hpp>
#include <dbot_ros/utils/ros_camera_data_provider.hpp>

namespace dbot
{
RosCameraDataProvider::RosCameraDataProvider(const ros::NodeHandle& nh,
    const std::string& camera_info_topic,
    const std::string& depth_image_topic,
    const CameraData::Resolution& native_res,
    int downsampling_factor,
    double timeout)
    : nh_(nh),
      camera_info_topic_(camera_info_topic),
      depth_image_topic_(depth_image_topic),
      native_resolution_(native_res),
      downsampling_factor_(downsampling_factor),
      timeout_(timeout)
{
}

Eigen::MatrixXd RosCameraDataProvider::depth_image() const
{
    sensor_msgs::Image::ConstPtr ros_image =
        ros::topic::waitForMessage<sensor_msgs::Image>(
            depth_image_topic_, nh_, ros::Duration(timeout_));

    auto image = ri::Ros2Eigen<double>(*ros_image, downsampling_factor_);

    return image;
}

Eigen::VectorXd RosCameraDataProvider::depth_image_vector() const
{
    sensor_msgs::Image::ConstPtr ros_image =
        ros::topic::waitForMessage<sensor_msgs::Image>(
            depth_image_topic_, nh_, ros::Duration(timeout_));

    auto image = ri::Ros2EigenVector<double>(*ros_image, downsampling_factor_);

    return image;
}

Eigen::Matrix3d RosCameraDataProvider::camera_matrix() const
{
    auto cam_mat =
        ri::GetCameraMatrix<double>(camera_info_topic_, nh_, timeout_);
    cam_mat.topLeftCorner(2, 3) /= downsampling_factor();
    return cam_mat;
}

std::string RosCameraDataProvider::frame_id() const
{
    return ri::GetCameraFrame<double>(camera_info_topic_, nh_, timeout_);
}

int RosCameraDataProvider::downsampling_factor() const
{
    return downsampling_factor_;
}

CameraData::Resolution RosCameraDataProvider::native_resolution() const
{
   return native_resolution_;
}
}
