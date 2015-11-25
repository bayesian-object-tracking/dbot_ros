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

#include <ros/package.h>

#include <fl/util/profiling.hpp>

#include <dbot_ros_pkg/trackers/rbc_particle_filter_object_tracker.hpp>
#include <dbot_ros_pkg/utils/ros_interface.hpp>
#include <dbot_ros_pkg/utils/object_file_reader.hpp>

#include <dbot_ros_pkg/utils/cloud_visualizer.hpp>

#include <dbot/model/state_transition/orientation_transition_function.hpp>

#include <boost/filesystem.hpp>

namespace bot
{
RbcParticleFilterObjectTracker::RbcParticleFilterObjectTracker(
    const Parameters& param)
    : param_(param)
{
    std::cout << "constructing rbc filter" << std::endl;

    std::cout << "use_gpu" << param.use_gpu << std::endl;
    std::cout << "angular_acceleration_sigma" << param.angular_acceleration_sigma << std::endl;
    std::cout << "angular_sigma " << param.angular_sigma << std::endl;
    std::cout << "damping " << param.damping << std::endl;
    std::cout << "downsampling_factor " << param.downsampling_factor << std::endl;
    std::cout << "evaluation_count " << param.evaluation_count << std::endl;
    std::cout << "linear_acceleration_sigma " << param.linear_acceleration_sigma << std::endl;
    std::cout << "initial_occlusion_prob " << param.initial_occlusion_prob << std::endl;
    std::cout << "linear_sigma " << param.linear_sigma << std::endl;
    std::cout << "max_kl_divergence " << param.max_kl_divergence << std::endl;
    std::cout << "max_sample_count " << param.max_sample_count << std::endl;
    std::cout << "model_sigma " << param.model_sigma << std::endl;
    std::cout << "p_occluded_occluded " << param.p_occluded_occluded << std::endl;
    std::cout << "p_occluded_visible " << param.p_occluded_visible << std::endl;
    std::cout << "sigma_factor " << param.sigma_factor << std::endl;
    std::cout << "tail_weight " << param.tail_weight << std::endl;
    std::cout << "use_new_process " << param.use_new_process << std::endl;
    std::cout << "velocity_factor " << param.velocity_factor << std::endl;

    object_publisher_ =
        ros::NodeHandle("~")
            .advertise<visualization_msgs::Marker>("object_model", 0);

    std::cout << "finishing filter construction" << std::endl;
}

void RbcParticleFilterObjectTracker::Initialize(
    std::vector<Eigen::VectorXd> initial_states,
    const sensor_msgs::Image& ros_image,
    Eigen::Matrix3d camera_matrix)
{
    boost::mutex::scoped_lock lock(mutex_);

    // convert camera matrix and image to desired format
    camera_matrix.topLeftCorner(2, 3) /= double(downsampling_factor_);
    Observation image = ri::Ros2Eigen<Scalar>(ros_image, downsampling_factor_);

    /// read parameters ********************************************************
    double delta_time = 0.033;

    std::cout << "sampling blocks: " << std::endl;
    dbot::hf::PrintVector(param_.sampling_blocks);

    /// load object mesh *******************************************************
    std::vector<std::vector<Eigen::Vector3d>> vertices(param_.object_names.size());
    std::vector<std::vector<std::vector<int>>> triangle_indices(
        param_.object_names.size());
    for (size_t i = 0; i < param_.object_names.size(); i++)
    {
        std::string object_model_path = ros::package::getPath("dbot_ros_pkg") +
                                        "/object_models/" + param_.object_names[i] +
                                        ".obj";
        ObjectFileReader file_reader;
        file_reader.set_filename(object_model_path);
        file_reader.Read();

        vertices[i] = *file_reader.get_vertices();
        triangle_indices[i] = *file_reader.get_indices();
    }

    /// compute object centers *************************************************
    centers_.resize(vertices.size());
    for (size_t i = 0; i < vertices.size(); i++)
    {
        centers_[i] = Eigen::Vector3d::Zero();
        for (size_t j = 0; j < vertices[i].size(); j++)
        {
            centers_[i] += vertices[i][j];
        }
        centers_[i] /= double(vertices[i].size());
    }

    /// switch coordinate system ***********************************************
    for (size_t i = 0; i < vertices.size(); i++)
    {
        for (size_t j = 0; j < vertices[i].size(); j++)
        {
            vertices[i][j] -= centers_[i];
        }
    }
    for (size_t i = 0; i < initial_states.size(); i++)
    {
        State state = initial_states[i];

        for (size_t j = 0; j < state.count(); j++)
        {
            state.component(j).position() +=
                state.component(j).orientation().rotation_matrix() *
                centers_[j];
        }

        initial_states[i] = state;
    }

    /// initialize cpu observation model ***************************************
    boost::shared_ptr<ObservationModel> observation_model;
#ifndef BUILD_GPU
    param_.use_gpu = false;
#endif

    if (!param_.use_gpu)
    {
        // cpu obseration model
        boost::shared_ptr<dbot::KinectPixelObservationModel>
            kinect_pixel_observation_model(
                new dbot::KinectPixelObservationModel(param_.tail_weight,
                                                      param_.model_sigma,
                                                      param_.sigma_factor));

        boost::shared_ptr<dbot::OcclusionProcessModel> occlusion_process(
            new dbot::OcclusionProcessModel(param_.p_occluded_visible,
                                            param_.p_occluded_occluded));

        boost::shared_ptr<dbot::RigidBodyRenderer> renderer(
            new dbot::RigidBodyRenderer(vertices, triangle_indices));

        observation_model = boost::shared_ptr<ObservationModelCPUType>(
            new ObservationModelCPUType(camera_matrix,
                                        image.rows(),
                                        image.cols(),
                                        initial_states.size(),
                                        renderer,
                                        kinect_pixel_observation_model,
                                        occlusion_process,
                                        param_.initial_occlusion_prob,
                                        delta_time));
    }

    /// initialize gpu observation model ***************************************
    else
    {
#ifdef BUILD_GPU

        /// \todo this is suboptimal to hardcode the path here.
        std::string vertex_shader_path =
            ros::package::getPath("dbot") + "/src/dbot/model/observation/" +
            "gpu/shaders/" + "VertexShader.vertexshader";

        std::string fragment_shader_path =
            ros::package::getPath("dbot") + "/src/dbot/model/observation/" +
            "gpu/shaders/" + "FragmentShader.fragmentshader";

        // gpu obseration model
        boost::shared_ptr<ObservationModelGPUType> gpu_observation_model(
            new ObservationModelGPUType(camera_matrix,
                                        image.rows(),
                                        image.cols(),
                                        param_.max_sample_count,
                                        vertices,
                                        triangle_indices,
                                        vertex_shader_path,
                                        fragment_shader_path,
                                        param_.initial_occlusion_prob,
                                        delta_time,
                                        param_.p_occluded_visible,
                                        param_.p_occluded_occluded,
                                        param_.tail_weight,
                                        param_.model_sigma,
                                        param_.sigma_factor));

        observation_model = gpu_observation_model;
#endif
    }
    //    std::cout << "initialized observation omodel " << std::endl;

    /// initialize process model ***********************************************
    Eigen::MatrixXd linear_acceleration_covariance =
        Eigen::MatrixXd::Identity(3, 3) *
        pow(double(param_.linear_acceleration_sigma), 2);
    Eigen::MatrixXd angular_acceleration_covariance =
        Eigen::MatrixXd::Identity(3, 3) *
        pow(double(param_.angular_acceleration_sigma), 2);

    OldStateTransition old_process(delta_time, param_.object_names.size());

    //    std::cout << "setting center shizzles " << std::endl;
    for (size_t i = 0; i < param_.object_names.size(); i++)
    {
        old_process.Parameters(i,
                               Eigen::Vector3d::Zero(),
                               param_.damping,
                               linear_acceleration_covariance,
                               angular_acceleration_covariance);
    }

    //    std::cout << "initialized process model " << std::endl;

    /// initialize NEW process model *******************************************
    NewStateTransition new_process(12, 6);
    auto A = new_process.create_dynamics_matrix();
    A.setIdentity();
    //    A.bottomLeftCorner(6,6).setZero();
    A.topRightCorner(6, 6).setIdentity();
    A.rightCols(6) *= param_.velocity_factor;
    //    A.bottomRightCorner(6,6) = A.topRightCorner(6,6);
    new_process.dynamics_matrix(A);

    auto B = new_process.create_noise_matrix();
    B.setZero();
    B.block<3, 3>(6, 0) = Eigen::Matrix3d::Identity() * param_.linear_sigma;
    B.block<3, 3>(9, 3) = Eigen::Matrix3d::Identity() * param_.angular_sigma;
    B.topRows(6) = B.bottomRows(6);
    new_process.noise_matrix(B);

    auto C = new_process.create_input_matrix();
    C.setZero();
    new_process.input_matrix(C);

    //    std::cout << "dynamics: " << std::endl <<
    //    new_process.dynamics_matrix() <<
    //    std::endl;
    //    std::cout << "noise: " << std::endl << new_process.noise_matrix() <<
    //    std::endl;
    //    std::cout << "input: " << std::endl << new_process.input_matrix() <<
    //    std::endl;

    // exit(-1);

    boost::shared_ptr<StateTransition> process;

    if (param_.use_new_process)
    {
        process = boost::shared_ptr<StateTransition>(
            new NewStateTransition(new_process));
        param_.sampling_blocks.resize(1);
        param_.sampling_blocks[0].resize(12);
        for (size_t i = 0; i < 12; i++)
        {
            param_.sampling_blocks[0][i] = i;
        }
    }
    else
    {
        process = boost::shared_ptr<StateTransition>(
            new OldStateTransition(old_process));
    }

    /// initialize filter ******************************************************
    filter_ =
        boost::shared_ptr<FilterType>(new FilterType(process,
                                                     observation_model,
                                                     param_.sampling_blocks,
                                                     param_.max_kl_divergence));

    std::vector<State> multi_body_samples(initial_states.size());
    for (size_t i = 0; i < multi_body_samples.size(); i++)
    {
        multi_body_samples[i] = initial_states[i];
    }

    filter_->set_particles(multi_body_samples);

    filter_->filter(image,
                    StateTransition::Input::Zero(param_.object_names.size() * 6));
    filter_->resample(param_.evaluation_count / param_.sampling_blocks.size());

    /// convert to a differential reperesentation ******************************
    State mean = filter_->belief().mean();
    filter_->observation_model()->default_poses().recount(mean.count());
    for (size_t i = 0; i < mean.count(); i++)
    {
        auto pose = filter_->observation_model()->default_poses().component(i);
        auto delta = mean.component(i);
        pose.orientation() = delta.orientation() * pose.orientation();
        pose.position() = delta.position() + pose.position();
    }

    for (size_t i_part = 0; i_part < filter_->belief().size(); i_part++)
    {
        State& state = filter_->belief().location(i_part);
        for (size_t i_obj = 0; i_obj < mean.count(); i_obj++)
        {
            state.component(i_obj).position() =
                state.component(i_obj).position() -
                mean.component(i_obj).position();

            state.component(i_obj).orientation() =
                state.component(i_obj).orientation() *
                mean.component(i_obj).orientation().inverse();

            // this needs to be set to zero, because as we switch coordinate
            // system, the linear velocity changes, since it has to account for
            // part of the angular velocity.
            state.component(i_obj).linear_velocity() = Eigen::Vector3d::Zero();
            state.component(i_obj).angular_velocity() = Eigen::Vector3d::Zero();
        }
    }
}

Eigen::VectorXd RbcParticleFilterObjectTracker::Filter(
    const sensor_msgs::Image& ros_image)
{
    boost::mutex::scoped_lock lock(mutex_);

    //    if (std::isnan(last_measurement_time_))
    //        last_measurement_time_ = ros_image.header.stamp.toSec();
    //    Scalar delta_time = ros_image.header.stamp.toSec() -
    //    last_measurement_time_;
    //    last_measurement_time_ = ros_image.header.stamp.toSec();
    //    std::cout << "actual delta time " << delta_time << std::endl;
    // convert image
    Observation image = ri::Ros2Eigen<Scalar>(ros_image, downsampling_factor_);

    /// filter *****************************************************************
    //    INIT_PROFILING;
    filter_->filter(image,
                    StateTransition::Input::Zero(param_.object_names.size() * 6));
    //    MEASURE("-----------------> total time for filtering");

    /// convert to a differential reperesentation ******************************
    State mean_delta = filter_->belief().mean();
    filter_->observation_model()->default_poses().recount(mean_delta.count());
    for (size_t i = 0; i < mean_delta.count(); i++)
    {
        auto pose = filter_->observation_model()->default_poses().component(i);
        auto delta = mean_delta.component(i);
        pose.orientation() = delta.orientation() * pose.orientation();
        pose.position() = delta.position() + pose.position();
    }

    for (size_t i_part = 0; i_part < filter_->belief().size(); i_part++)
    {
        State& state = filter_->belief().location(i_part);
        for (size_t i_obj = 0; i_obj < mean_delta.count(); i_obj++)
        {
            state.component(i_obj).position() -=
                mean_delta.component(i_obj).position();
            state.component(i_obj).orientation() -=
                mean_delta.component(i_obj).orientation();
        }
    }

    /// visualize the mean state ***********************************************
    State mean = filter_->belief().mean();
    for (size_t i = 0; i < mean.count(); i++)
    {
        auto pose_0 =
            filter_->observation_model()->default_poses().component(i);
        auto state = mean.component(i);

        state.position() = state.position() + pose_0.position();
        state.orientation() = state.orientation() * pose_0.orientation();
    }

    // switch coordinate system
    for (size_t j = 0; j < mean.count(); j++)
    {
        mean.component(j).position() -=
            mean.component(j).orientation().rotation_matrix() * centers_[j];
    }

    for (size_t i = 0; i < param_.object_names.size(); i++)
    {
        std::string object_model_path =
            "package://dbot_ros_pkg/object_models/" + param_.object_names[i] + ".obj";

        ri::PublishMarker(mean.component(i).homogeneous().cast<float>(),
                          ros_image.header,
                          object_model_path,
                          object_publisher_,
                          i,
                          1,
                          0,
                          0);
    }
    return mean;
}
}
