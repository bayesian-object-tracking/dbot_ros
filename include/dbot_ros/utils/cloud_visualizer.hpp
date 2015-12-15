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
 * \file cloud_visualizer.hpp
 * \author Manuel Wuthrich (manuel.wuthrich@gmail.com)
 */

#include <Eigen/Dense>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>

#include <vector>

namespace vis
{


class CloudVisualizer
{
public:
	CloudVisualizer();
	~CloudVisualizer();

    void add_cloud(const pcl::PointCloud<pcl::PointXYZRGB>& point_cloud,
                   const Eigen::Matrix3f R = Eigen::Matrix3f::Identity(),
                   const Eigen::Vector3f t = Eigen::Vector3f::Zero());

    void add_cloud(const pcl::PointCloud<pcl::PointXYZ>& point_cloud,
                   const Eigen::Matrix3f R = Eigen::Matrix3f::Identity(),
                   const Eigen::Vector3f t = Eigen::Vector3f::Zero());

    void add_cloud(const std::vector<Eigen::Vector3d>  &points,
                   const Eigen::Matrix3d R = Eigen::Matrix3d::Identity(),
                   const Eigen::Vector3d t = Eigen::Vector3d::Zero());

    void add_cloud(const std::vector<Eigen::Vector3f> &points_,
                   const Eigen::Matrix3f R = Eigen::Matrix3f::Identity(),
                   const Eigen::Vector3f t = Eigen::Vector3f::Zero());
    void add_cloud(const std::vector<Eigen::Vector3i> &points_,
                   const Eigen::Matrix3f R = Eigen::Matrix3f::Identity(),
                   const Eigen::Vector3f t = Eigen::Vector3f::Zero());


    template<typename Scalar>
    void add_cloud(const Eigen::Matrix<Eigen::Matrix<Scalar, 3, 1>, -1, -1> &points,
                   const Eigen::Matrix<Scalar, 3, 1> t = Eigen::Matrix<Scalar, 3, 1>::Zero(),
                   const Eigen::Matrix<Scalar, 3, 3> R = Eigen::Matrix<Scalar, 3, 3>::Identity())
    {
        pcl::PointCloud<pcl::PointXYZRGB> point_cloud;

        // Fill in the point_cloud data
        point_cloud.width    = points.size();
        point_cloud.height   = 1;
        point_cloud.is_dense = false;
        point_cloud.points.resize(point_cloud.width * point_cloud.height);

        for(size_t row = 0; row < points.rows(); row++)
            for(size_t col = 0; col < points.cols(); col++)
            {
                size_t index = row*points.cols() + col;
                point_cloud.points[index].x = points(row, col)(0);
                point_cloud.points[index].y = points(row, col)(1);
                point_cloud.points[index].z = points(row, col)(2);

                point_cloud.points[index].r = 100;
                point_cloud.points[index].g = 50;
                point_cloud.points[index].b = 200;
            }
        add_cloud(point_cloud, R.template cast<float>(), t.template cast<float>());
    }


	void add_line(Eigen::Vector3d from, Eigen::Vector3d to);
	void add_point(Eigen::Vector3d point, Eigen::Matrix4d transform_ = Eigen::Matrix4d::Identity());

	void set_origin_to_mean();

	void show(bool loop = true);
	void reset();

private:
    std::vector<pcl::PointCloud<pcl::PointXYZRGB> > point_clouds_;
    std::vector<Eigen::Matrix4f> poses_;
    std::vector<Eigen::Vector3f> lines_;
    std::vector<Eigen::Vector3f> points_;
    Eigen::Matrix4f origin_;
    pcl::visualization::PCLVisualizer pcl_visualizer_;
};

}
