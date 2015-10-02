/*************************************************************************
This software allows for filtering in high-dimensional observation and
state spaces, as described in

M. Wuthrich, P. Pastor, M. Kalakrishnan, J. Bohg, and S. Schaal.
Probabilistic Object Tracking using a Range Camera
IEEE/RSJ Intl Conf on Intelligent Robots and Systems, 2013

In a publication based on this software pleace cite the above reference.


Copyright (C) 2014  Manuel Wuthrich

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*************************************************************************/

#ifndef POSE_TRACKING_INTERFACE_TRACKERS_OBJECT_TRACKER_HPP
#define POSE_TRACKING_INTERFACE_TRACKERS_OBJECT_TRACKER_HPP

#include <boost/thread/mutex.hpp>

#include <Eigen/Dense>

#include <vector>
#include <string>

// ros stuff
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include <dbot/rao_blackwell_coordinate_particle_filter.hpp>
#include <dbot/models/process_models/brownian_object_motion_model.hpp>
#include <dbot/models/observation_models/kinect_image_observation_model_cpu.hpp>
#ifdef BUILD_GPU
#include <dbot/models/observation_models/kinect_image_observation_model_gpu/kinect_image_observation_model_gpu.hpp>
#endif

#include <fl/model/process/linear_state_transition_model.hpp>
#include <fl/model/process/interface/state_transition_function.hpp>


#include <fl/util/math/pose_vector.hpp>
#include <fl/util/math/composed_vector.hpp>

class MultiObjectTracker
{
public:
    typedef Eigen::VectorXd StateVector;
    typedef fl::PoseBlock<StateVector> StateBlock;

//    typedef fl::ComposedVector<StateBlock, StateVector> State;

    typedef dbot::FreeFloatingRigidBodiesState<>  State;
    typedef State::Scalar                       Scalar;


    typedef Eigen::Matrix<fl::Real, -1, 1> Input;

    typedef fl::StateTransitionFunction<State, State, Input> StateTransition;


    typedef fl::LinearStateTransitionModel<State, Input> NewStateTransition;

//    typedef StateTransition ProcessModel;

    typedef dbot::BrownianObjectMotionModel<State>        OldStateTransition;





    typedef dbot::KinectImageObservationModelCPU<Scalar,
                                                State>  ObservationModelCPUType;
#ifdef BUILD_GPU
    typedef dbot::KinectImageObservationModelGPU<State>   ObservationModelGPUType;
#endif

    typedef ObservationModelCPUType::Base ObservationModel;
    typedef ObservationModelCPUType::Observation Observation;

    typedef dbot::RBCoordinateParticleFilter<StateTransition, ObservationModel> FilterType;


    typedef typename Eigen::Transform<fl::Real, 3, Eigen::Affine> Affine;


    MultiObjectTracker();

    void Reset(std::vector<Eigen::VectorXd> initial_states, const sensor_msgs::Image &ros_image);

    void Initialize(std::vector<Eigen::VectorXd> initial_states,
                    const sensor_msgs::Image& ros_image,
                    Eigen::Matrix3d camera_matrix);

    Eigen::VectorXd Filter(const sensor_msgs::Image& ros_image);

private:
    Scalar last_measurement_time_;

    boost::mutex mutex_;
    ros::NodeHandle node_handle_;
    ros::Publisher object_publisher_;

    boost::shared_ptr<FilterType> filter_;

    // parameters
//    std::string object_model_uri_;
//    std::string object_model_path_;
    std::vector<std::string> object_names_;
    int downsampling_factor_;

    std::vector<Eigen::Vector3d> centers_;

    std::vector<Affine> default_poses_;
};


#endif

