/*
 * This is part of the fl library, a C++ Bayesian filtering library
 * (https://github.com/filtering-library)
 *
 * Copyright (c) 2015 Max Planck Society,
 * 				 Autonomous Motion Department,
 * 			     Institute for Intelligent Systems
 *
 * This Source Code Form is subject to the terms of the GPL License (MIT).
 * A copy of the license can be found in the LICENSE file distributed with this
 * source code.
 */

#pragma once

#include <Eigen/Dense>

#include <vector>
#include <string>
#include <memory>
#include <mutex>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include <dbot/rao_blackwell_coordinate_particle_filter.hpp>
#include <dbot/model/state_transition/brownian_object_motion_model.hpp>
#include <dbot/model/observation/kinect_image_observation_model_cpu.hpp>

#ifdef BUILD_GPU
#include <dbot/model/observation/gpu/kinect_image_observation_model_gpu.hpp>
#endif

#include <fl/model/process/linear_state_transition_model.hpp>
#include <fl/model/process/interface/state_transition_function.hpp>

#include <osr/pose_vector.hpp>
#include <osr/composed_vector.hpp>

namespace bot
{


/**
 * \brief RbcParticleFilterObjectTracker
 *
 * Yaml config file:
 * \code
 * object:
 *  package:
 *  directory:
 *  files: [  ]
 *  sampling_blocks: [ [0, 1, 2], [3, 4], [6, 7] ]
 *
 * \endcode
 */
class RbcParticleFilterObjectTracker
{
public:
    struct Parameters
    {
        std::vector<std::string> object_names;
        int downsampling_factor;

        std::vector<std::vector<size_t> > sampling_blocks;
        bool use_gpu;
        bool use_new_process;
        int evaluation_count;
        int max_sample_count;
        double max_kl_divergence;
        double initial_occlusion_prob;
        double p_occluded_visible;
        double p_occluded_occluded;
        double linear_acceleration_sigma;
        double angular_acceleration_sigma;
        double damping;
        double velocity_factor;
        double linear_sigma;
        double angular_sigma;
        double tail_weight;
        double model_sigma;
        double sigma_factor;
    };

public:
    typedef Eigen::VectorXd StateVector;
    typedef osr::PoseBlock<StateVector> StateBlock;

    typedef osr::FreeFloatingRigidBodiesState<> State;
    typedef State::Scalar Scalar;

    typedef Eigen::Matrix<fl::Real, -1, 1> Input;

    typedef fl::StateTransitionFunction<State, State, Input> StateTransition;

    typedef fl::LinearStateTransitionModel<State, Input> NewStateTransition;

    typedef dbot::BrownianObjectMotionModel<State> OldStateTransition;

    typedef dbot::KinectImageObservationModelCPU<Scalar, State>
        ObservationModelCPUType;

#ifdef BUILD_GPU
    typedef dbot::KinectImageObservationModelGPU<State> ObservationModelGPUType;
#endif

    typedef ObservationModelCPUType::Base ObservationModel;
    typedef ObservationModelCPUType::Observation Observation;

    typedef dbot::RBCoordinateParticleFilter<StateTransition, ObservationModel>
        FilterType;

    typedef typename Eigen::Transform<fl::Real, 3, Eigen::Affine> Affine;

    RbcParticleFilterObjectTracker(const Parameters& param);

    void Reset(std::vector<Eigen::VectorXd> initial_states,
               const sensor_msgs::Image& ros_image);

    void Initialize(std::vector<Eigen::VectorXd> initial_states,
                    const sensor_msgs::Image& ros_image,
                    Eigen::Matrix3d camera_matrix);

    Eigen::VectorXd Filter(const sensor_msgs::Image& ros_image);

private:
    std::mutex mutex_;
    std::shared_ptr<FilterType> filter_;
    ros::Publisher object_publisher_;

    // parameters
    //    std::string object_model_uri_;
    //    std::string object_model_path_;

    std::vector<Eigen::Vector3d> centers_;
    std::vector<Affine> default_poses_;
    Parameters param_;
};
}
