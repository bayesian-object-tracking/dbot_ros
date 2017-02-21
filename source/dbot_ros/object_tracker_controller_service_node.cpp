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
 * \file  object_tracker_controller_service_node.cpp
 * \date April 2016
 * \author Jan Issac (jan.issac@gmail.com)
 */

#include <Eigen/Dense>
#include <ctime>
#include <dbot/builder/particle_tracker_builder.h>
#include <dbot/camera_data.h>
#include <dbot/pose/free_floating_rigid_bodies_state.h>
#include <dbot/simple_wavefront_object_loader.h>
#include <dbot/tracker/particle_tracker.h>
#include <dbot_ros/object_tracker_publisher.h>
#include <dbot_ros/object_tracker_ros.h>
#include <dbot_ros/util/interactive_marker_initializer.h>
#include <dbot_ros/util/ros_camera_data_provider.h>
#include <dbot_ros/util/ros_interface.h>
#include <dbot_ros_msgs/FindObject.h>
#include <dbot_ros_msgs/RunObjectTracker.h>
#include <dbot_ros_msgs/TrackObject.h>
#include <fl/util/profiling.hpp>
#include <fstream>
#include <memory>
#include <ros/package.h>
#include <ros/ros.h>
#include <thread>

static bool running = false;
static std::thread tracker_thread;
static std::shared_ptr<opi::InteractiveMarkerInitializer> object_initializer;

static ros::ServiceClient object_finder_service_client;
static ros::ServiceClient object_tracker_service_client;

static dbot_ros_msgs::TrackObjectRequest last_req;

bool stop_object_tracker()
{
    ros::NodeHandle nh_prv("~");
    auto objects_package   = ri::read<std::string>("objects/package", nh_prv);
    auto objects_directory = ri::read<std::string>("objects/directory", nh_prv);

    dbot_ros_msgs::RunObjectTracker run_object_tracker_srv;
    run_object_tracker_srv.request.object_state.ori.package = objects_package;
    run_object_tracker_srv.request.object_state.ori.directory =
        objects_directory;
    run_object_tracker_srv.request.object_state.ori.name = "stop";
    run_object_tracker_srv.request.object_state.name     = "stop";


    ROS_INFO("Stopping object tracker ...");
    if (!object_tracker_service_client.call(run_object_tracker_srv))
    {
        ROS_ERROR("Stopping object tracker failed.");
        return false;
    }
    return true;
}

void marker_callback(const geometry_msgs::PoseArray& poses)
{
    // stop tracker first
    if (!stop_object_tracker())
    {
        return;
    }

    ROS_INFO(
        "Object pose set. Confirm the pose by clicking on the "
        "interactive marker!");

    if (!object_initializer->wait_for_object_poses())
    {
        ROS_INFO("Setting object poses was interrupted.");
        return;
    }

    ROS_INFO("Object pose confirmed. Triggering object tracker ...");

    geometry_msgs::PoseStamped pose;
    pose.pose = object_initializer->poses()[0];


    // find new pose
    ros::NodeHandle nh_prv("~");
    auto objects_package   = ri::read<std::string>("objects/package", nh_prv);
    auto objects_directory = ri::read<std::string>("objects/directory", nh_prv);


    // trigger tracking
    dbot_ros_msgs::RunObjectTracker run_object_tracker_srv;
    run_object_tracker_srv.request.object_state.ori.package = objects_package;
    run_object_tracker_srv.request.object_state.ori.directory =
        objects_directory;
    run_object_tracker_srv.request.object_state.ori.name =
        last_req.object_name + ".obj";
    run_object_tracker_srv.request.object_state.name = last_req.object_name;
    run_object_tracker_srv.request.object_state.pose = pose;
    if (!object_tracker_service_client.call(run_object_tracker_srv))
    {
        ROS_ERROR("Running object tracker for '%s' failed.",
                  last_req.object_name.c_str());
    }
}

bool track_object_srv(dbot_ros_msgs::TrackObjectRequest& req,
                      dbot_ros_msgs::TrackObjectResponse& res)
{
    //    if (req.object_name == "stop")
    //    {
    //       return stop_object_tracker();
    //    }

    object_initializer->delete_poses_update_callback();

    //    // stop tracker first
    //    if (!stop_object_tracker())
    //    {
    //        return false;
    //   }

    // find new pose
    ros::NodeHandle nh_prv("~");
    auto objects_package   = ri::read<std::string>("objects/package", nh_prv);
    auto objects_directory = ri::read<std::string>("objects/directory", nh_prv);

    geometry_msgs::PoseStamped pose;

    if (req.auto_detect)
    {
        dbot_ros_msgs::FindObject find_object_srv;
        find_object_srv.request.object_ori.package   = objects_package;
        find_object_srv.request.object_ori.directory = objects_directory;
        find_object_srv.request.object_ori.name      = req.object_name + ".obj";
        if (!object_finder_service_client.call(find_object_srv))
        {
            ROS_ERROR("Finding object '%s' failed.", req.object_name.c_str());
            return false;
        }
        ROS_INFO_STREAM("Object found:\n" << find_object_srv.response);

        if (!req.auto_confirm)
        {
            object_initializer->set_object(
                find_object_srv.response.found_object.ori.package,
                find_object_srv.response.found_object.ori.directory,
                find_object_srv.response.found_object.ori.name,
                find_object_srv.response.found_object.pose.pose,
                false);
        }

        pose = find_object_srv.response.found_object.pose;
    }
    else
    {
        object_initializer->set_objects(objects_package,
                                        objects_directory,
                                        {req.object_name + ".obj"},
                                        {},
                                        true,
                                        !req.auto_confirm);

        pose.pose = object_initializer->poses()[0];
    }

    if (!req.auto_confirm)
    {
        ROS_INFO(
            "Object pose set. Confirm the pose by clicking on the "
            "interactive marker!");

        if (!object_initializer->wait_for_object_poses())
        {
            ROS_INFO("Setting object poses was interrupted.");
            return false;
        }

        ROS_INFO("Object pose confirmed. Triggering object tracker ...");

        pose.pose = object_initializer->poses()[0];
    }

    // trigger tracking
    dbot_ros_msgs::RunObjectTracker run_object_tracker_srv;
    run_object_tracker_srv.request.object_state.ori.package = objects_package;
    run_object_tracker_srv.request.object_state.ori.directory =
        objects_directory;
    run_object_tracker_srv.request.object_state.ori.name =
        req.object_name + ".obj";
    run_object_tracker_srv.request.object_state.name = req.object_name;
    run_object_tracker_srv.request.object_state.pose = pose;
    if (!object_tracker_service_client.call(run_object_tracker_srv))
    {
        ROS_ERROR("Running object tracker for '%s' failed.",
                  req.object_name.c_str());
        return false;
    }

    last_req = req;
    object_initializer->poses_update_callback(marker_callback);

    return true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "object_tracker_controller_service");
    ros::NodeHandle nh;
    ros::NodeHandle nh_prv("~");


    auto object_tracker_controller_service_name =
        ri::read<std::string>("object_tracker_controller_service_name", nh_prv);
    auto object_tracker_service_name =
        ri::read<std::string>("object_tracker_service_name", nh_prv);
    auto object_finder_service_name =
        ri::read<std::string>("object_finder_service_name", nh_prv);

    auto camera_info_topic = ri::read<std::string>("camera_info_topic", nh_prv);

    auto camera_frame = ri::get_camera_frame(camera_info_topic, nh, 5.);

    if (camera_frame.empty())
    {
        ROS_ERROR("Cannot obtain camera frame.");
        return 1;
    }

    object_initializer =
        std::make_shared<opi::InteractiveMarkerInitializer>(camera_frame);

    auto srv = nh.advertiseService(object_tracker_controller_service_name,
                                   track_object_srv);

    object_finder_service_client =
        nh.serviceClient<dbot_ros_msgs::FindObject>(object_finder_service_name);
    object_finder_service_client.waitForExistence();

    object_tracker_service_client =
        nh.serviceClient<dbot_ros_msgs::RunObjectTracker>(
            object_tracker_service_name);
    object_tracker_service_client.waitForExistence();

    ROS_INFO("Object tracker controller service up and running.");
    ROS_INFO("Waiting for requests...");

    ros::spin();

    return 0;
}
