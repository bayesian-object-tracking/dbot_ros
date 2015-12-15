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
 * \file pcl_interface.hpp
 * \author Manuel Wuthrich (manuel.wuthrich@gmail.com)
 */
#include <Eigen/Core>

#include <vector>

#include <pcl/point_cloud.h>

#include <sensor_msgs/PointCloud2.h>

#include <osr/free_floating_rigid_bodies_state.hpp>

namespace pi
{

template<typename PointT, typename MatrixT>
void Pcl2Eigen(
        const pcl::PointCloud<PointT> &point_cloud,
        std::vector<Eigen::Matrix<MatrixT, 3, 1> > &vector,
        size_t &n_rows, size_t &n_cols);

template<typename T>
void Ros2Std(
        const sensor_msgs::PointCloud2& ros_cloud,
        const size_t& n_downsampling,
        std::vector<T>& observations,
        size_t& n_rows,
        size_t& n_cols);

template<typename PointT, typename MatrixT>
std::vector<Eigen::Matrix<MatrixT, 3, 1> > Pcl2Eigen(const pcl::PointCloud<PointT> &point_cloud);

template<typename PointT, typename MatrixT>
void Eigen2Pcl( const Eigen::Matrix<Eigen::Matrix<MatrixT, 3, 1>, -1, -1>& eigen,
                pcl::PointCloud<PointT>& pcl);

template<typename PointT, typename MatrixT>
void Eigen2Pcl(
        const std::vector<Eigen::Matrix<MatrixT, 3, 1> > &vector,
        const size_t &n_rows, const size_t &n_cols,
        pcl::PointCloud<PointT> &point_cloud);

template<typename PointT, typename MatrixT>
pcl::PointCloud<PointT> Eigen2Pcl(
        const std::vector<Eigen::Matrix<MatrixT, 3, 1> > &vector,
        const size_t &n_rows = 0, const size_t &n_cols = 1);

template<typename PointT, typename MatrixT>
void PointsOnPlane(
        const pcl::PointCloud<PointT>& input_cloud,
        pcl::PointCloud<PointT>& output_cloud,
        Eigen::Matrix<MatrixT, 4, 1>& table_plane,
        const bool& keep_organized = false,
        const float& z_min = 0.3,  const float& z_max = 2.0,
        const float& y_min = -1.0, const float& y_max = 1.0,
        const float& x_min = -1.0, const float& x_max = 1.0,
        const float& min_table_dist = 0.01, const float& max_table_dist = 0.4,
        const float& grid_size = 0.02);

template<typename MatrixT>
void PointsOnPlane(
        const std::vector<Eigen::Matrix<MatrixT, 3, 1> >& input_points,
        const size_t &input_rows, const size_t &input_cols,
        std::vector<Eigen::Matrix<MatrixT, 3, 1> >& output_points,
        size_t& output_rows, size_t& output_cols,
        Eigen::Matrix<MatrixT, 4, 1>& table_plane,
        const bool& keep_organized = false,
        const float& z_min = 0.3,  const float& z_max = 1.5,
        const float& y_min = -0.4, const float& y_max = 0.4,
        const float& x_min = -0.3, const float& x_max = 0.3,
        const float& min_table_dist = 0.01, const float& max_table_dist = 0.3,
        const float& grid_size = 0.02);


template <typename PointT> void Cluster(
        const pcl::PointCloud<PointT>& input_point_cloud,
        std::vector<pcl::PointCloud<PointT> >& clusters,
        const float& cluster_delta_ = 0.01,
        const size_t& min_cluster_size_ = 200,
        const float& erosion_pixel_radius = 2);

template <typename MatrixT> void Cluster(
        const std::vector<Eigen::Matrix<MatrixT, 3, 1> > &input_points,
        const size_t& input_rows,
        const size_t& input_cols,
        std::vector<std::vector<Eigen::Matrix<MatrixT, 3, 1> > > &output_points,
        const float& cluster_delta = 0.01,
        const size_t& min_cluster_size = 200,
        const float& erosion_pixel_radius = 2);

template <typename PointT> void FindCylinder(
        const pcl::PointCloud<PointT> &input_point_cloud,
        pcl::PointCloud<PointT> &inliers,
        pcl::PointCloud<PointT> &outliers,
        Eigen::Matrix<float, 7, 1> &coefficients,
        const float r_min,
        const float r_max);

// this function creates some samples around clusters on a plane. it assumes
// that when the object is standing on the table, the origin coincides with the
// table plane and z points upwards
template<typename Scalar> std::vector<osr::FreeFloatingRigidBodiesState<-1>::State>
SampleTableClusters(const std::vector<Eigen::Matrix<Scalar,3,1> >& points,
                    const size_t& n_rows, const size_t& n_cols,
                    const size_t& sample_count);


template<typename Scalar> std::vector<osr::FreeFloatingRigidBodiesState<-1>::State>
SampleTableClusters(const Eigen::Matrix<Eigen::Matrix<Scalar,3,1>, -1, -1>& points,
                    const size_t& sample_count);

}
