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

#include <Eigen/Dense>

#include <fstream>
#include <ctime>
#include <memory>

#include <ros/ros.h>
#include <ros/package.h>

#include <fl/util/profiling.hpp>

#include <opi/interactive_marker_initializer.hpp>
#include <osr/free_floating_rigid_bodies_state.hpp>
#include <dbot/util/camera_data.hpp>
#include <dbot/tracker/rbc_particle_filter_object_tracker.hpp>
#include <dbot/tracker/builder/rbc_particle_filter_tracker_builder.hpp>

#include <dbot_ros_pkg/utils/ros_interface.hpp>
#include <dbot_ros_pkg/utils/ros_camera_data_provider.hpp>

typedef dbot::RbcParticleFilterObjectTracker Tracker;

/**
 * \brief Represents the RBC PF tracker node
 */
class TrackerNode
{
public:
    typedef Tracker::State State;
    typedef Tracker::Obsrv Obsrv;

public:
    /**
     * \brief Creates a TrackerNode
     */
    TrackerNode(const std::shared_ptr<Tracker>& tracker,
                const dbot::ObjectResourceIdentifier& ori)
        : tracker_(tracker), node_handle_("~"), ori_(ori)
    {
        object_publisher_ = node_handle_.advertise<visualization_msgs::Marker>(
            "object_model", 0);
    }

    /**
     * \brief Tracking callback function which is invoked whenever a new image
     *        is available
     */
    void tracking_callback(const sensor_msgs::Image& ros_image)
    {
        auto image = ri::Ros2Eigen<typename Obsrv::Scalar>(
            ros_image, tracker_->camera_data().downsampling_factor());

        auto mean_state = tracker_->track(image);

        publish(mean_state, ros_image.header);
    }

private:
    /**
     * \brief Publishes the object markers and it's pose
     */
    void publish(const State& state, const std_msgs::Header& header) const
    {
        for (int i = 0; i < ori_.count_meshes(); i++)
        {
            ri::PublishMarker(state.component(i).homogeneous().cast<float>(),
                              header,
                              ori_.mesh_uri(i),
                              object_publisher_,
                              i,
                              1,
                              0,
                              0);
        }
    }

private:
    std::shared_ptr<Tracker> tracker_;
    ros::NodeHandle node_handle_;
    ros::Publisher object_publisher_;
    dbot::ObjectResourceIdentifier ori_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_filter");
    ros::NodeHandle nh("~");

    /* ------------------------------ */
    /* - Parameters                 - */
    /* ------------------------------ */
    // tracker's main parameter container
    dbot::RbcParticleFilterTrackerBuilder::Parameters param;

    // camera data
    dbot::CameraData::Resolution resolution;
    std::string camera_info_topic;
    std::string depth_image_topic;
    int downsampling_factor;

    // object data
    std::string object_package;
    std::string object_directory;
    std::vector<std::string> object_meshes;

    /* ------------------------------ */
    /* - Read out parameters        - */
    /* ------------------------------ */
    // get object parameters
    nh.getParam("object/meshes", object_meshes);
    nh.getParam("object/package", object_package);
    nh.getParam("object/directory", object_directory);

    // get filter parameters
    nh.getParam("use_gpu", param.use_gpu);
    nh.getParam("evaluation_count", param.evaluation_count);
    nh.getParam("max_kl_divergence", param.max_kl_divergence);

    // observation model parameters
    nh.getParam("max_sample_count", param.obsrv.max_sample_count);
    nh.getParam("p_occluded_visible", param.obsrv.p_occluded_visible);
    nh.getParam("p_occluded_occluded", param.obsrv.p_occluded_occluded);
    nh.getParam("initial_occlusion_prob", param.obsrv.initial_occlusion_prob);
    nh.getParam("tail_weight", param.obsrv.tail_weight);
    nh.getParam("model_sigma", param.obsrv.model_sigma);
    nh.getParam("sigma_factor", param.obsrv.sigma_factor);
    param.obsrv.delta_time = 1. / 30.;

    // process model parameters
    nh.getParam("linear_acceleration_sigma",
                param.process.linear_acceleration_sigma);
    nh.getParam("angular_acceleration_sigma",
                param.process.angular_acceleration_sigma);
    nh.getParam("damping", param.process.damping);
    param.process.part_count = object_meshes.size();
    param.process.delta_time = 1. / 30.;

    // camera parameters
    nh.getParam("camera_info_topic", camera_info_topic);
    nh.getParam("depth_image_topic", depth_image_topic);
    nh.getParam("downsampling_factor", downsampling_factor);
    nh.getParam("resolution/width", resolution.width);
    nh.getParam("resolution/height", resolution.height);

    // setup object resource identifier
    param.ori.package_path(ros::package::getPath(object_package));
    param.ori.directory(object_directory);
    param.ori.meshes(object_meshes);

    /* ------------------------------ */
    /* - Setup camera data          - */
    /* ------------------------------ */
    // setup camera data
    auto camera_data_provider = std::shared_ptr<dbot::CameraDataProvider>(
        new dbot::RosCameraDataProvider(nh,
                                        camera_info_topic,
                                        depth_image_topic,
                                        resolution,
                                        downsampling_factor,
                                        2.0));
    dbot::CameraData camera_data(camera_data_provider);

    /* ------------------------------ */
    /* - Initialize interactively   - */
    /* ------------------------------ */
    opi::InteractiveMarkerInitializer object_initializer(camera_data.frame_id(),
                                                         param.ori.package(),
                                                         param.ori.directory(),
                                                         param.ori.meshes());
    if (!object_initializer.wait_for_object_poses())
    {
        ROS_INFO("Setting object poses was interrupted.");
        return 0;
    }

    auto initial_ros_poses = object_initializer.poses();
    std::vector<Tracker::State> initial_poses;
    for (auto& ros_pose : initial_ros_poses)
    {
        initial_poses.push_back(ri::to_pose_velocity_vector(ros_pose));
    }

    /* ------------------------------ */
    /* - Create the tracker         - */
    /* ------------------------------ */
    auto tracker_builder = dbot::RbcParticleFilterTrackerBuilder(
        param, camera_data);

    auto tracker = tracker_builder.build();
    tracker->initialize(initial_poses,
                        param.evaluation_count);

    /* ------------------------------ */
    /* - Create and run tracker     - */
    /* - node                       - */
    /* ------------------------------ */
    TrackerNode tracker_node(tracker, param.ori);
    ros::Subscriber subscriber = nh.subscribe(
        depth_image_topic, 1, &TrackerNode::tracking_callback, &tracker_node);

    ros::spin();

    return 0;
}
