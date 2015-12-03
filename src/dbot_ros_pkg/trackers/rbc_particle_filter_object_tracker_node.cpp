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

#include <dbot_ros_pkg/utils/ros_interface.hpp>
#include <dbot_ros_pkg/utils/ros_camera_data_provider.hpp>
#include <dbot_ros_pkg/trackers/rbc_particle_filter_object_tracker.hpp>

using namespace bot;

typedef RbcParticleFilterObjectTracker Tracker;

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
    TrackerNode(const std::shared_ptr<Tracker>& tracker)
        : tracker_(tracker), node_handle_("~")
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

    /**
     * \brief Provides access to the tracker parameters
     */
    const Tracker::Parameters& param() const { return tracker_->param(); }
private:
    /**
     * \brief Publishes the object markers and it's pose
     */
    void publish(const State& state, const std_msgs::Header& header) const
    {
        for (int i = 0; i < param().ori.count_meshes(); i++)
        {
            ri::PublishMarker(state.component(i).homogeneous().cast<float>(),
                              header,
                              param().ori.mesh_uri(i),
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
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_filter");
    ros::NodeHandle nh("~");

    /* ------------------------------ */
    /* - Parameters                 - */
    /* ------------------------------ */
    // tracker's main parameter container
    Tracker::Parameters param;

    // camera data
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
    nh.getParam("max_sample_count", param.max_sample_count);
    nh.getParam("initial_occlusion_prob", param.initial_occlusion_prob);
    nh.getParam("p_occluded_visible", param.p_occluded_visible);
    nh.getParam("p_occluded_occluded", param.p_occluded_occluded);
    nh.getParam("linear_acceleration_sigma", param.linear_acceleration_sigma);
    nh.getParam("angular_acceleration_sigma", param.angular_acceleration_sigma);
    nh.getParam("damping", param.damping);
    nh.getParam("velocity_factor", param.velocity_factor);
    nh.getParam("linear_sigma", param.linear_sigma);
    nh.getParam("angular_sigma", param.angular_sigma);
    nh.getParam("tail_weight", param.tail_weight);
    nh.getParam("model_sigma", param.model_sigma);
    nh.getParam("sigma_factor", param.sigma_factor);

    // camera parameters
    nh.getParam("camera_info_topic", camera_info_topic);
    nh.getParam("depth_image_topic", depth_image_topic);
    nh.getParam("downsampling_factor", downsampling_factor);

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
                                        downsampling_factor,
                                        2.0));
    dbot::CameraData camera_data(camera_data_provider);

    /* ------------------------------ */
    /* - Get initial poses          - */
    /* - interactively 				- */
    /* ------------------------------ */
    opi::InteractiveMarkerInitializer object_initializer(camera_data.frame_id(),
                                                         param.ori.package(),
                                                         param.ori.directory(),
                                                         param.ori.meshes());

    if (!object_initializer.wait_for_all_object_poses())
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
    // get observations from camera
    auto tracker = std::make_shared<Tracker>(param, initial_poses, camera_data);

    /* ------------------------------ */
    /* - Create and run tracker     - */
    /* - node                       - */
    /* ------------------------------ */
    TrackerNode tracker_node(tracker);
    ros::Subscriber subscriber = nh.subscribe(
        depth_image_topic, 1, &TrackerNode::tracking_callback, &tracker_node);

    ros::spin();

    return 0;
}
