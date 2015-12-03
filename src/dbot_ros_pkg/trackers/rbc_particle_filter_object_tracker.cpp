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

namespace bot
{
RbcParticleFilterObjectTracker::RbcParticleFilterObjectTracker(
    const Parameters& param,
    const std::vector<State>& initial_states,
    const dbot::CameraData& camera_data)
    : param_(param), camera_data_(camera_data)

{
    initialize(initial_states);
}

void RbcParticleFilterObjectTracker::initialize(
    const std::vector<State>& initial_states)
{
    std::lock_guard<std::mutex> lock(mutex_);

    Obsrv image = camera_data_.depth_image();
    Eigen::Matrix3d camera_matrix = camera_data_.camera_matrix();

    /// read parameters ********************************************************

    double delta_time = 0.033;

    /// load object mesh *******************************************************
    std::vector<std::vector<Eigen::Vector3d>> vertices(
        param_.ori.count_meshes());
    std::vector<std::vector<std::vector<int>>> triangle_indices(
        param_.ori.count_meshes());
    for (size_t i = 0; i < param_.ori.count_meshes(); i++)
    {
        ObjectFileReader file_reader;
        file_reader.set_filename(param_.ori.mesh_path(i));
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

    std::vector<State> states = initial_states;

    for (size_t i = 0; i < initial_states.size(); i++)
    {
        State state = initial_states[i];

        for (size_t j = 0; j < state.count(); j++)
        {
            state.component(j).position() +=
                state.component(j).orientation().rotation_matrix() *
                centers_[j];
        }

        states.push_back(state);
    }

    /// initialize cpu observation model ***************************************
    std::shared_ptr<ObservationModel> observation_model;
#ifndef BUILD_GPU
    param_.use_gpu = false;
#endif

    if (!param_.use_gpu)
    {
        // cpu obseration model
        std::shared_ptr<dbot::KinectPixelObservationModel>
            kinect_pixel_observation_model(
                new dbot::KinectPixelObservationModel(param_.tail_weight,
                                                      param_.model_sigma,
                                                      param_.sigma_factor));

        std::shared_ptr<dbot::OcclusionProcessModel> occlusion_process(
            new dbot::OcclusionProcessModel(param_.p_occluded_visible,
                                            param_.p_occluded_occluded));

        std::shared_ptr<dbot::RigidBodyRenderer> renderer(
            new dbot::RigidBodyRenderer(vertices, triangle_indices));

        observation_model = std::shared_ptr<ObservationModelCPUType>(
            new ObservationModelCPUType(camera_matrix,
                                        image.rows(),
                                        image.cols(),
                                        states.size(),
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
        std::shared_ptr<ObservationModelGPUType> gpu_observation_model(
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

    OldStateTransition old_process(delta_time, param_.ori.count_meshes());

    //    std::cout << "setting center shizzles " << std::endl;
    for (size_t i = 0; i < param_.ori.count_meshes(); i++)
    {
        old_process.Parameters(i,
                               Eigen::Vector3d::Zero(),
                               param_.damping,
                               linear_acceleration_covariance,
                               angular_acceleration_covariance);
    }

    std::shared_ptr<StateTransition> process =
        std::shared_ptr<StateTransition>(new OldStateTransition(old_process));

    /// initialize filter ******************************************************
    filter_ = std::shared_ptr<FilterType>(new FilterType(
        process,
        observation_model,
        create_sampling_blocks(
            param_.ori.count_meshes(),
            process->noise_dimension() / param_.ori.count_meshes()),
        param_.max_kl_divergence));

    std::vector<State> multi_body_samples(states.size());
    for (size_t i = 0; i < multi_body_samples.size(); i++)
    {
        multi_body_samples[i] = states[i];
    }

    filter_->set_particles(multi_body_samples);

    filter_->filter(
        image, StateTransition::Input::Zero(param_.ori.count_meshes() * 6));
    filter_->resample(param_.evaluation_count / param_.ori.count_meshes());

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

std::vector<std::vector<size_t>>
RbcParticleFilterObjectTracker::create_sampling_blocks(int blocks,
                                                       int block_size) const
{
    std::vector<std::vector<size_t>> sampling_blocks(param_.ori.count_meshes());
    for (int i = 0; i < blocks; ++i)
    {
        for (int k = 0; k < block_size; ++k)
        {
            sampling_blocks[i].push_back(i * block_size + k);
        }
    }

    return sampling_blocks;
}

auto RbcParticleFilterObjectTracker::track(const Obsrv& image) -> State
{
    std::lock_guard<std::mutex> lock(mutex_);

    /// filter *****************************************************************
    //    INIT_PROFILING;
    filter_->filter(
        image, StateTransition::Input::Zero(param_.ori.count_meshes() * 6));
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

    return mean;
}
}
