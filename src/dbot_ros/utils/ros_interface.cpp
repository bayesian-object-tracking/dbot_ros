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
 * \file ros_interface.hpp
 * \date 2014
 * \author Manuel Wuthrich (manuel.wuthrich@gmail.com)
 * \author Jan Issac (jan.issac@gmail.com)
 */

#include <dbot_ros/utils/ros_interface.hpp>
#include <dbot_ros_msgs/ObjectState.h>

template void ri::read_parameter(const std::string& path,
                                double& parameter,
                                ros::NodeHandle node_handle);

template void ri::read_parameter(const std::string& path,
                                int& parameter,
                                ros::NodeHandle node_handle);

template void ri::read_parameter(const std::string& path,
                                std::string& parameter,
                                ros::NodeHandle node_handle);

template <>
void ri::read_parameter<std::vector<std::string>>(
    const std::string& path,
    std::vector<std::string>& parameter,
    ros::NodeHandle node_handle)
{
    XmlRpc::XmlRpcValue ros_parameter;
    node_handle.getParam(path, ros_parameter);
    parameter.resize(ros_parameter.size());
    for (size_t i = 0; i < parameter.size(); i++)
        parameter[i] = std::string(ros_parameter[i]);
}

template <>
void ri::read_parameter<std::vector<double>>(const std::string& path,
                                            std::vector<double>& parameter,
                                            ros::NodeHandle node_handle)
{
    XmlRpc::XmlRpcValue ros_parameter;
    node_handle.getParam(path, ros_parameter);
    parameter.resize(ros_parameter.size());
    for (size_t i = 0; i < parameter.size(); i++)
        parameter[i] = double(ros_parameter[i]);
}

template <>
void ri::read_parameter<std::vector<std::vector<int>>>(
    const std::string& path,
    std::vector<std::vector<int>>& parameter,
    ros::NodeHandle node_handle)
{
    XmlRpc::XmlRpcValue ros_parameter;
    node_handle.getParam(path, ros_parameter);
    parameter.resize(ros_parameter.size());
    for (size_t i = 0; i < parameter.size(); i++)
    {
        parameter[i].resize(ros_parameter[i].size());
        for (size_t j = 0; j < parameter[i].size(); j++)
            parameter[i][j] = int(ros_parameter[i][j]);
    }
}



void ri::publish_marker(const Eigen::Matrix4d &H,
                        const std::string& frame_id,
                        const ros::Time& stamp,
                        const std::string &object_model_path,
                        const ros::Publisher& pub,
                        const int &marker_id,
                        const float &r,
                        const float &g,
                        const float &b,
                        const float &a,
                        const std::string &ns)
{
    visualization_msgs::Marker marker;

    marker.pose = to_ros_pose(H);

    marker.header.frame_id = frame_id;
    marker.header.stamp = stamp;
    marker.ns = ns;
    marker.id = marker_id;

    marker.mesh_resource = object_model_path;
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = a;

    marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    marker.action = visualization_msgs::Marker::ADD;

    pub.publish(marker);
}

void ri::publish_pose(const Eigen::Matrix4d H,
                      const std::string& frame_id,
                      const ros::Time& stamp,
                      const std::string& object_name,
                      const std::string& object_directory,
                      const std::string& object_package,
                      const ros::Publisher& pub)
{
    dbot_ros_msgs::ObjectState object_state_message;

    object_state_message.pose.pose = to_ros_pose(H);
    object_state_message.pose.header.frame_id = frame_id;
    object_state_message.pose.header.stamp = stamp;


    object_state_message.ori.name = object_name;
    object_state_message.ori.directory = object_directory;
    object_state_message.ori.package = object_package;

    size_t ind = object_name.find_last_of(".");
    object_state_message.name = object_name.substr(0, ind);

    pub.publish(object_state_message);
}

















