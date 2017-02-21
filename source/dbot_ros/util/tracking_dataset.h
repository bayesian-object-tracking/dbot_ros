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
 * \file tracking_dataset.h
 * \author Manuel Wuthrich (manuel.wuthrich@gmail.com)
 */

#pragma once

#include <Eigen/Dense>
#include <boost/filesystem.hpp>
#include <fstream>
#include <message_filters/simple_filter.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/JointState.h>
#include <tf/tfMessage.h>

class DataFrame
{
public:
    sensor_msgs::Image::ConstPtr image_;
    sensor_msgs::CameraInfo::ConstPtr info_;
    sensor_msgs::JointState::ConstPtr ground_truth_joints_;
    sensor_msgs::JointState::ConstPtr noisy_joints_;
    tf::tfMessage::ConstPtr gt_tf_;
    tf::tfMessage::ConstPtr gt_tf_fixed_;
    Eigen::VectorXd ground_truth_;
    Eigen::VectorXd deviation_;

    DataFrame(const sensor_msgs::Image::ConstPtr& image,
              const sensor_msgs::CameraInfo::ConstPtr& info,
              const Eigen::VectorXd& ground_truth = Eigen::VectorXd(),
              const Eigen::VectorXd& deviation    = Eigen::VectorXd());

    DataFrame(const sensor_msgs::Image::ConstPtr& image,
              const sensor_msgs::CameraInfo::ConstPtr& info,
              const tf::tfMessage::ConstPtr gt_tf,
              const Eigen::VectorXd& ground_truth = Eigen::VectorXd(),
              const Eigen::VectorXd& deviation    = Eigen::VectorXd());

    DataFrame(const sensor_msgs::Image::ConstPtr& image,
              const sensor_msgs::CameraInfo::ConstPtr& info,
              const sensor_msgs::JointState::ConstPtr& ground_truth_joints,
              const sensor_msgs::JointState::ConstPtr& noisy_joints,
              const Eigen::VectorXd& ground_truth = Eigen::VectorXd(),
              const Eigen::VectorXd& deviation    = Eigen::VectorXd());

    DataFrame(const sensor_msgs::Image::ConstPtr& image,
              const sensor_msgs::CameraInfo::ConstPtr& info,
              const sensor_msgs::JointState::ConstPtr& ground_truth_joints,
              const sensor_msgs::JointState::ConstPtr& noisy_joints,
              const tf::tfMessage::ConstPtr& tf,
              const tf::tfMessage::ConstPtr& fixed_tf,
              const Eigen::VectorXd& ground_truth = Eigen::VectorXd(),
              const Eigen::VectorXd& deviation    = Eigen::VectorXd());
};

template <class M>
class BagSubscriber : public message_filters::SimpleFilter<M>
{
public:
    void newMessage(const boost::shared_ptr<M const>& msg)
    {
        this->signalMessage(msg);
    }
};

class TrackingDataset
{
public:
    enum DataType
    {
        GROUND_TRUTH = 1,
        DEVIATION
    };

    TrackingDataset(const std::string& path);
    ~TrackingDataset();

    //    void AddFrame(const sensor_msgs::Image::ConstPtr& image,
    //                  const sensor_msgs::CameraInfo::ConstPtr& info,
    //                  const Eigen::VectorXd& ground_truth = Eigen::VectorXd(),
    //                  const Eigen::VectorXd& deviation = Eigen::VectorXd());

    void AddFrame(const sensor_msgs::Image::ConstPtr& image,
                  const sensor_msgs::CameraInfo::ConstPtr& info);

    void AddFrame(const sensor_msgs::Image::ConstPtr& image,
                  const sensor_msgs::CameraInfo::ConstPtr& info,
                  const tf::tfMessage::ConstPtr& gt_tf);

    sensor_msgs::Image::ConstPtr GetImage(const size_t& index);

    sensor_msgs::CameraInfo::ConstPtr GetInfo(const size_t& index);

    //    pcl::PointCloud<pcl::PointXYZ>::ConstPtr GetPointCloud(const size_t&
    //    index);

    Eigen::Matrix3d GetCameraMatrix(const size_t& index);

    Eigen::VectorXd GetGroundTruth(const size_t& index);

    size_t Size();

    void Load();

    void Store();

protected:
    bool LoadTextFile(const char* filename, DataType type);
    bool StoreTextFile(const char* filename, DataType type);

    std::vector<DataFrame> data_;
    const boost::filesystem::path path_;

    const std::string image_topic_;
    const std::string info_topic_;
    const std::string observations_filename_;
    const std::string ground_truth_filename_;

private:
    const double admissible_delta_time_;  // admissible time difference in s for
                                          // comparing time stamps
};
