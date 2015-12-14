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

template void ri::ReadParameter(const std::string& path,
                                double& parameter,
                                ros::NodeHandle node_handle);

template void ri::ReadParameter(const std::string& path,
                                int& parameter,
                                ros::NodeHandle node_handle);

template void ri::ReadParameter(const std::string& path,
                                std::string& parameter,
                                ros::NodeHandle node_handle);

template<>
void ri::ReadParameter<std::vector<std::string> >(const std::string& path,
                                               std::vector<std::string>& parameter,
                                               ros::NodeHandle node_handle)
{
    XmlRpc::XmlRpcValue ros_parameter;
    node_handle.getParam(path, ros_parameter);
    parameter.resize(ros_parameter.size());
    for(size_t i = 0; i < parameter.size(); i++)
        parameter[i] = std::string(ros_parameter[i]);
}


template<>
void ri::ReadParameter<std::vector<double> >(const std::string& path,
                                               std::vector<double>& parameter,
                                               ros::NodeHandle node_handle)
{
    XmlRpc::XmlRpcValue ros_parameter;
    node_handle.getParam(path, ros_parameter);
    parameter.resize(ros_parameter.size());
    for(size_t i = 0; i < parameter.size(); i++)
        parameter[i] = double(ros_parameter[i]);
}

template<>
void ri::ReadParameter<std::vector<std::vector<size_t> > >(const std::string& path,
                                                        std::vector<std::vector<size_t> >& parameter,
                                                        ros::NodeHandle node_handle)
{
    XmlRpc::XmlRpcValue ros_parameter;
    node_handle.getParam(path, ros_parameter);
    parameter.resize(ros_parameter.size());
    for(size_t i = 0; i < parameter.size(); i++)
    {
        parameter[i].resize(ros_parameter[i].size());
        for(size_t j = 0; j < parameter[i].size(); j++)
            parameter[i][j] = int(ros_parameter[i][j]);
    }
}

void ri::PublishMarker(const Eigen::Matrix3f R, const Eigen::Vector3f t,
        std_msgs::Header header,
        std::string object_model_path,
        const ros::Publisher &pub,
        int marker_id, float r, float g, float b, float a,
        std::string ns)
{

    Eigen::Quaternion<float> q(R);

    geometry_msgs::PoseWithCovarianceStamped pose;
    pose.header =  header;
    pose.pose.pose.position.x = t(0);
    pose.pose.pose.position.y = t(1);
    pose.pose.pose.position.z = t(2);

    pose.pose.pose.orientation.x = q.x();
    pose.pose.pose.orientation.y = q.y();
    pose.pose.pose.orientation.z = q.z();
    pose.pose.pose.orientation.w = q.w();


    visualization_msgs::Marker marker;
    marker.header.frame_id =  pose.header.frame_id;
    marker.header.stamp =  pose.header.stamp;
    marker.ns = ns;
    marker.id = marker_id;
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = a;

    marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose = pose.pose.pose;

    marker.mesh_resource = object_model_path;

    pub.publish(marker);
}

void ri::PublishMarker(const Eigen::Matrix4f H,
        std_msgs::Header header,
        std::string object_model_path,
        const ros::Publisher &pub,
        int marker_id, float r, float g, float b, float a,
        std::string ns)
{
    PublishMarker(H.topLeftCorner(3,3),
                  H.topRightCorner(3,1),
                  header,
                  object_model_path, pub,
                  marker_id, r, g, b, a,
                  ns);
}

void ri::PublishPoints(const std_msgs::Header header,
        const ros::Publisher &pub,
        const std::vector<Eigen::Vector3f> points,
        std::vector<float> colors,
        const Eigen::Matrix3f R,
        Eigen::Vector3f t)
{
    // if no color has been given we set it to some value -----------------------------
    if(colors.size() == 0)
        colors = std::vector<float> (points.size(), 1);

    // renormalize colors -----------------------------
    float max = -std::numeric_limits<float>::max();
    float min = std::numeric_limits<float>::max();
    for(int i = 0; i < int(colors.size()); i++)
    {
        min = colors[i] < min ? colors[i] : min;
        max = colors[i] > max ? colors[i] : max;
    }
    if(min == max) min = 0;

    pcl::PointCloud<pcl::PointXYZRGB> point_cloud;
    point_cloud.header = header;
    point_cloud.width    = points.size(); point_cloud.height   = 1; point_cloud.is_dense = false;
    point_cloud.points.resize (point_cloud.width * point_cloud.height);

    for (int point_index = 0; point_index < int(points.size()); ++point_index)
    {
        Eigen::Vector3f new_point = R*points[point_index] + t;
        point_cloud.points[point_index].x = new_point(0);
        point_cloud.points[point_index].y = new_point(1);
        point_cloud.points[point_index].z = new_point(2);

        point_cloud.points[point_index].r = (colors[point_index]-min)/(max-min) * 255.;
        point_cloud.points[point_index].g = 0.;
        point_cloud.points[point_index].b = (1 - (colors[point_index]-min)/(max-min)) * 255.;
    }
    sensor_msgs::PointCloud2 point_cloud2;
    pcl::toROSMsg(point_cloud, point_cloud2);
    pub.publish(point_cloud2);
}
