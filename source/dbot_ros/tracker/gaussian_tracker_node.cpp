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
 * \file gaussian_tracker_node.cpp
 * \date December 2015
 * \author Jan Issac (jan.issac@gmail.com)
 */

#include <Eigen/Dense>
#include <ctime>
#include <dbot/builder/gaussian_tracker_builder.h>
#include <dbot/camera_data.h>
#include <dbot/pose/free_floating_rigid_bodies_state.h>
#include <dbot/tracker/gaussian_tracker.h>
#include <dbot_ros/object_tracker_publisher.h>
#include <dbot_ros/object_tracker_ros.h>
#include <dbot_ros/util/interactive_marker_initializer.h>
#include <dbot_ros/util/ros_camera_data_provider.h>
#include <dbot_ros/util/ros_interface.h>
#include <dbot_ros_msgs/ObjectState.h>
#include <fl/util/profiling.hpp>
#include <fstream>
#include <memory>
#include <ros/package.h>
#include <ros/ros.h>

typedef dbot::GaussianTracker Tracker;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "gaussian_tracker");
    ros::NodeHandle nh("~");

    /* ------------------------------ */
    /* - Parameters                 - */
    /* ------------------------------ */
    // tracker's main parameter container
    dbot::GaussianTrackerBuilder::Parameters params;

    // camera data
    dbot::CameraData::Resolution resolution;
    std::string camera_info_topic;
    std::string depth_image_topic;
    int downsampling_factor;

    // object data
    std::string object_package;
    std::string object_directory;
    std::vector<std::string> object_meshes;

    // parameter shorthand prefix
    std::string pre = "gaussian_filter/";

    /* ------------------------------ */
    /* - Read out parameters        - */
    /* ------------------------------ */
    // get object parameters
    /// \todo nh.getParam does not check whether the parameter exists in the
    /// config file. this is dangerous, we should use ri::read instead

    nh.getParam("object/meshes", object_meshes);
    nh.getParam("object/package", object_package);
    nh.getParam("object/directory", object_directory);

    params.ori.package_path(ros::package::getPath(object_package));
    params.ori.directory(object_directory);
    params.ori.meshes(object_meshes);

    // get filter parameters
    nh.getParam(pre + "unscented_transform/alpha", params.ut_alpha);
    nh.getParam(pre + "moving_average_update_rate",
                params.moving_average_update_rate);
    nh.getParam(pre + "center_object_frame", params.center_object_frame);

    nh.getParam(pre + "observation/tail_weight",
                params.observation.tail_weight);
    nh.getParam(pre + "observation/bg_depth", params.observation.bg_depth);
    nh.getParam(pre + "observation/fg_noise_std",
                params.observation.fg_noise_std);
    nh.getParam(pre + "observation/bg_noise_std",
                params.observation.bg_noise_std);
    nh.getParam(pre + "observation/uniform_tail_max",
                params.observation.uniform_tail_max);
    nh.getParam(pre + "observation/uniform_tail_min",
                params.observation.uniform_tail_min);

    // state transition parameters
    nh.getParam(pre + "object_transition/linear_sigma_x",
                params.object_transition.linear_sigma_x);
    nh.getParam(pre + "object_transition/linear_sigma_y",
                params.object_transition.linear_sigma_y);
    nh.getParam(pre + "object_transition/linear_sigma_z",
                params.object_transition.linear_sigma_z);

    nh.getParam(pre + "object_transition/angular_sigma_x",
                params.object_transition.angular_sigma_x);
    nh.getParam(pre + "object_transition/angular_sigma_y",
                params.object_transition.angular_sigma_y);
    nh.getParam(pre + "object_transition/angular_sigma_z",
                params.object_transition.angular_sigma_z);

    nh.getParam(pre + "object_transition/velocity_factor",
                params.object_transition.velocity_factor);
    params.object_transition.part_count = object_meshes.size();

    // camera parameters
    nh.getParam("camera_info_topic", camera_info_topic);
    nh.getParam("depth_image_topic", depth_image_topic);
    nh.getParam("downsampling_factor", downsampling_factor);
    nh.getParam("resolution/width", resolution.width);
    nh.getParam("resolution/height", resolution.height);

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
    auto camera_data = std::make_shared<dbot::CameraData>(camera_data_provider);

    // finally, set number of pixels
    params.observation.sensors = camera_data->pixels();

    /* ------------------------------ */
    /* - Initialize interactively   - */
    /* ------------------------------ */
    opi::InteractiveMarkerInitializer object_initializer(
        camera_data->frame_id(),
        params.ori.package(),
        params.ori.directory(),
        params.ori.meshes(),
        {},
        true);
    if (!object_initializer.wait_for_object_poses())
    {
        ROS_INFO("Setting object poses was interrupted.");
        return 0;
    }

    auto initial_ros_poses = object_initializer.poses();
    std::vector<Tracker::State> initial_poses;
    initial_poses.push_back(Tracker::State(params.ori.count_meshes()));
    int i = 0;
    for (auto& ros_pose : initial_ros_poses)
    {
        initial_poses[0].component(i++) = ri::to_pose_velocity_vector(ros_pose);
    }

    /* ------------------------------ */
    /* - Create the tracker         - */
    /* ------------------------------ */
    auto tracker_builder = dbot::GaussianTrackerBuilder(params, camera_data);

    auto tracker = tracker_builder.build();
    tracker->initialize(initial_poses);

    /* ------------------------------ */
    /* - Tracker publisher          - */
    /* ------------------------------ */
    int object_color[3];
    nh.getParam(pre + "object_color/R", object_color[0]);
    nh.getParam(pre + "object_color/G", object_color[1]);
    nh.getParam(pre + "object_color/B", object_color[2]);
    auto tracker_publisher = dbot::ObjectStatePublisher(
        params.ori, object_color[0], object_color[1], object_color[2]);

    /* ------------------------------ */
    /* - Create and run tracker     - */
    /* - node                       - */
    /* ------------------------------ */
    dbot::ObjectTrackerRos<dbot::GaussianTracker> ros_object_tracker(
        tracker, camera_data, params.ori.count_meshes());

    ros::Subscriber subscriber = nh.subscribe(
        depth_image_topic,
        1,
        &dbot::ObjectTrackerRos<dbot::GaussianTracker>::update_obsrv,
        &ros_object_tracker);

    while (ros::ok())
    {
        if (ros_object_tracker.run_once())
        {
            tracker_publisher.publish(
                ros_object_tracker.current_state_messages());
        }
        ros::spinOnce();
    }

    ros::spin();

    return 0;
}
