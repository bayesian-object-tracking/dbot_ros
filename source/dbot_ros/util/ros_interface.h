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
 * \file ros_interface.h
 * \author Manuel Wuthrich (manuel.wuthrich@gmail.com)
 */

#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <limits>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Header.h>
#include <string>
#include <visualization_msgs/Marker.h>

// to avoid typedef conflict
#define uint64 enchiladisima
#include <cv_bridge/cv_bridge.h>
#undef uint64

#include <XmlRpcException.h>
#include <dbot/pose/pose_vector.h>
#include <dbot/pose/pose_velocity_vector.h>
#include <sensor_msgs/Image.h>

namespace ri
{
/**
 * \brief Converts a ros pose message to dbot::PoseVector
 */
inline dbot::PoseVector to_pose_vector(const geometry_msgs::Pose& ros_pose)
{
    Eigen::Vector3d p;
    Eigen::Quaternion<double> q;
    p[0]  = ros_pose.position.x;
    p[1]  = ros_pose.position.y;
    p[2]  = ros_pose.position.z;
    q.w() = ros_pose.orientation.w;
    q.x() = ros_pose.orientation.x;
    q.y() = ros_pose.orientation.y;
    q.z() = ros_pose.orientation.z;

    dbot::PoseVector pose;
    pose.position() = p;
    pose.orientation().quaternion(q);

    return pose;
}

/**
 * \brief Converts a ros pose message to dbot::PoseVelocityVector
 */
inline dbot::PoseVelocityVector to_pose_velocity_vector(
    const geometry_msgs::Pose& ros_pose)
{
    Eigen::Vector3d p;
    Eigen::Quaternion<double> q;
    p[0]  = ros_pose.position.x;
    p[1]  = ros_pose.position.y;
    p[2]  = ros_pose.position.z;
    q.w() = ros_pose.orientation.w;
    q.x() = ros_pose.orientation.x;
    q.y() = ros_pose.orientation.y;
    q.z() = ros_pose.orientation.z;

    dbot::PoseVelocityVector pose;
    pose.position() = p;
    pose.orientation().quaternion(q);

    return pose;
}

inline geometry_msgs::Pose to_ros_pose(
    const dbot::PoseVelocityVector& pose_vector)
{
    auto p = pose_vector.position();
    auto q = pose_vector.orientation().quaternion();

    geometry_msgs::Pose ros_pose;
    ros_pose.position.x    = p[0];
    ros_pose.position.y    = p[1];
    ros_pose.position.z    = p[2];
    ros_pose.orientation.w = q.w();
    ros_pose.orientation.x = q.x();
    ros_pose.orientation.y = q.y();
    ros_pose.orientation.z = q.z();

    return ros_pose;
}

inline geometry_msgs::Twist to_ros_velocity(
    const dbot::PoseVelocityVector& pose_vector)
{
    auto v = pose_vector.linear_velocity();
    auto w = pose_vector.angular_velocity();

    geometry_msgs::Twist ros_velocity;
    ros_velocity.linear.x  = v[0];
    ros_velocity.linear.y  = v[1];
    ros_velocity.linear.z  = v[2];
    ros_velocity.angular.x = w[0];
    ros_velocity.angular.y = w[1];
    ros_velocity.angular.z = w[2];

    return ros_velocity;
}

inline geometry_msgs::Pose to_ros_pose(const Eigen::Matrix3d& R,
                                       const Eigen::Vector3d& t)
{
    dbot::PoseVelocityVector pose_vector;
    pose_vector.orientation().rotation_matrix(R);
    pose_vector.position() = t;
    return to_ros_pose(pose_vector);
}

template <typename Scalar>
Eigen::Matrix<Scalar, -1, -1> to_eigen_matrix(
    const sensor_msgs::Image& ros_image, const size_t& n_downsampling = 1)
{
  cv::Mat cv_image;
  // Kinect1/XTION use encoding 32FC1 and meters
  if(ros_image.encoding.compare("32FC1")==0)
    cv_image = cv_bridge::toCvCopy(ros_image)->image;
  else
    // Kinect2 uses encoding 16UC1 and millimeters and therefore has to be converted to 32FC1
    // TODO: Assumption that all depth sensors in encoding different from 32FC1 are in millimeters
    cv_bridge::toCvCopy(ros_image, ros_image.encoding)->image.convertTo(cv_image, CV_32F, 0.001);

  //  cv_bridge::toCvCopy(ros_image, sensor_msgs::image_encodings::TYPE_16UC1)->image.convertTo(cv_image, CV_32F, 0.001);
  //cv_bridge::toCvCopy(ros_image, ros_image.encoding)->image.convertTo(cv_image, CV_32F, 0.001);
  //  cv_bridge::toCvCopy(ros_image, ros_image.encoding)->image.convertTo(cv_image, CV_32F);

  
    size_t n_rows = cv_image.rows / n_downsampling;
    size_t n_cols = cv_image.cols / n_downsampling;
    Eigen::Matrix<Scalar, -1, -1> eigen_image(n_rows, n_cols);
    for (size_t row = 0; row < n_rows; row++)
        for (size_t col = 0; col < n_cols; col++)
            eigen_image(row, col) =
                cv_image.at<float>(row * n_downsampling, col * n_downsampling);

    return eigen_image;
}

template <typename Scalar>
Eigen::Matrix<Scalar, -1, 1> to_eigen_vector(
    const sensor_msgs::Image& ros_image, const size_t& n_downsampling = 1)
{
  cv::Mat cv_image;
  // Kinect1/XTION use encoding 32FC1 and meters
  if(ros_image.encoding.compare("32FC1")==0)
    cv_image = cv_bridge::toCvCopy(ros_image)->image;
  else
    // Kinect2 uses encoding 16UC1 and millimeters and therefore has to be converted to 32FC1
    // TODO: Assumption that all depth sensors in encoding different from 32FC1 are in millimeters
    cv_bridge::toCvCopy(ros_image, ros_image.encoding)->image.convertTo(cv_image, CV_32F, 0.001);
  
    size_t n_rows = cv_image.rows / n_downsampling;
    size_t n_cols = cv_image.cols / n_downsampling;

    Eigen::Matrix<Scalar, -1, 1> eigen_image(n_rows * n_cols, 1);
    for (size_t row = 0; row < n_rows; row++)
        for (size_t col = 0; col < n_cols; col++)
            eigen_image(row * n_cols + col) =
                cv_image.at<float>(row * n_downsampling, col * n_downsampling);

    return eigen_image;
}

/// access ros data ************************************************************
template <typename T>
struct CastFromRos;

template <typename T>
T cast_from_ros(XmlRpc::XmlRpcValue ros_parameter)
{
    return CastFromRos<T>::f(ros_parameter);
}

template <typename T>
struct CastFromRos
{
    static T f(XmlRpc::XmlRpcValue ros_parameter) { return T(ros_parameter); }
};

template <typename T>
struct CastFromRos<std::vector<T>>
{
    static std::vector<T> f(XmlRpc::XmlRpcValue ros_parameter)
    {
        std::vector<T> parameter(ros_parameter.size());
        for (size_t i = 0; i < parameter.size(); i++)
        {
            parameter[i] = cast_from_ros<T>(ros_parameter[i]);
        }
        return parameter;
    }
};


template <typename T>
struct CastFromRos<std::map<std::string, T>>
{
    static std::map<std::string, T> f(XmlRpc::XmlRpcValue ros_parameter)
    {
        std::map<std::string, T> parameter;

        for (auto entry : ros_parameter)
        {
            parameter[entry.first] = cast_from_ros<T>(entry.second);
        }

        return parameter;
    }
};

template <typename Parameter>
Parameter read(const std::string& path, ros::NodeHandle node_handle)
{
    XmlRpc::XmlRpcValue ros_parameter;

    if (!node_handle.getParam(path, ros_parameter))
    {
        ROS_ERROR_STREAM("Could not read parameter at " << path);
        throw;
    }

    return cast_from_ros<Parameter>(ros_parameter);
}


template <typename Scalar>
Eigen::Matrix<Scalar, 3, 3> get_camera_matrix(
    const std::string& camera_info_topic,
    ros::NodeHandle& node_handle,
    const Scalar& seconds)
{
    // TODO: Check if const pointer is valid before accessing memory
    sensor_msgs::CameraInfo::ConstPtr camera_info =
        ros::topic::waitForMessage<sensor_msgs::CameraInfo>(
            camera_info_topic, node_handle, ros::Duration(seconds));

    Eigen::Matrix<Scalar, 3, 3> camera_matrix =
        Eigen::Matrix<Scalar, 3, 3>::Zero();

    if (!camera_info)
    {
        // if not topic was received within <seconds>
        ROS_INFO("Waiting for camera info ...");
        return camera_matrix;
    }
    ROS_INFO("Camera info ... received");

    for (size_t col = 0; col < 3; col++)
        for (size_t row = 0; row < 3; row++)
            camera_matrix(row, col) = camera_info->K[col + row * 3];

    return camera_matrix;
}

template <typename Scalar>
std::string get_camera_frame(const std::string& camera_info_topic,
                             ros::NodeHandle& node_handle,
                             const Scalar& seconds)
{
    // TODO: Check if const pointer is valid before accessing memory
    sensor_msgs::CameraInfo::ConstPtr camera_info =
        ros::topic::waitForMessage<sensor_msgs::CameraInfo>(
            camera_info_topic, node_handle, ros::Duration(seconds));
    if (!camera_info)
    {
        // if not topic was received within <seconds>
        ROS_WARN(
            "CameraInfo wasn't received within %f seconds. Returning default "
            "Zero message.",
            seconds);
        return "";
    }

    return camera_info->header.frame_id;
}

void publish_marker(const geometry_msgs::PoseStamped& pose_stamped,
                    const std::string& object_model_path,
                    const ros::Publisher& pub,
                    const int& marker_id  = 0,
                    const float& r        = 0,
                    const float& g        = 0,
                    const float& b        = 1,
                    const float& a        = 1.0,
                    const std::string& ns = "object");
}
