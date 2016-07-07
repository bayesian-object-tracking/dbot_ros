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
 * \file rms_gaussian_filter_object_tracker_node.cpp
 * \date December 2015
 * \author Jan Issac (jan.issac@gmail.com)
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
#include <dbot/camera_data.hpp>
#include <dbot/tracker/rms_gaussian_filter_object_tracker.hpp>
#include <dbot/builder/rms_gaussian_filter_tracker_builder.hpp>

#include <dbot_ros_msgs/ObjectState.h>

#include <dbot_ros/object_tracker_ros.h>
#include <dbot_ros/object_tracker_publisher.h>
#include <dbot_ros/util/ros_interface.hpp>
#include <dbot_ros/util/ros_camera_data_provider.hpp>

typedef dbot::RmsGaussianFilterObjectTracker Tracker;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "rms_gaussian_filter_object_tracker");
    ros::NodeHandle nh("~");

    /* ------------------------------ */
    /* - Parameters                 - */
    /* ------------------------------ */
    // tracker's main parameter container
    dbot::RmsGaussianFilterTrackerBuilder::Parameters params;

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
    nh.getParam("object/meshes", object_meshes);
    nh.getParam("object/package", object_package);
    nh.getParam("object/directory", object_directory);

    params.ori.package_path(ros::package::getPath(object_package));
    params.ori.directory(object_directory);
    params.ori.meshes(object_meshes);

    // get filter parameters
    nh.getParam(pre + "unscented_transform/alpha", params.ut_alpha);
    nh.getParam(pre + "update_rate", params.update_rate);

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

    // linear state transition parameters
    nh.getParam(pre + "object_transition/linear_sigma",
                params.object_transition.linear_sigma);
    nh.getParam(pre + "object_transition/angular_sigma",
                params.object_transition.angular_sigma);
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
    auto tracker_builder =
        dbot::RmsGaussianFilterTrackerBuilder(params, camera_data);

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
    dbot::ObjectTrackerRos<dbot::RmsGaussianFilterObjectTracker>
        ros_object_tracker(tracker, camera_data, params.ori.count_meshes());

    ros::Subscriber subscriber =
        nh.subscribe(depth_image_topic,
                     1,
                     &dbot::ObjectTrackerRos<
                         dbot::RmsGaussianFilterObjectTracker>::update_obsrv,
                     &ros_object_tracker);

    while (ros::ok())
    {
        if (ros_object_tracker.run_once())
        {
            ROS_INFO_STREAM("Current pose estimate: "
                            << ros_object_tracker.current_state().transpose());
            tracker_publisher.publish(ros_object_tracker.current_poses());
        }
        else
        {
            ROS_INFO("Waiting for image ...");
        }
        ros::spinOnce();
    }

    ros::spin();

    return 0;
}
