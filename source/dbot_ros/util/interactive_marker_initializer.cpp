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
 * \file interactive_markers_initializer.cpp
 * \date October 2015
 * \author Jeannette Bohg (bohg.jeannette@googlemail.com)
 * \author Jan Issac (jan.issac@gmail.com)
 */

#include <boost/filesystem.hpp>
#include <dbot/pose/free_floating_rigid_bodies_state.h>
#include <dbot_ros/util/interactive_marker_initializer.h>
#include <fstream>
#include <ros/assert.h>

namespace opi
{
InteractiveMarkerInitializer::InteractiveMarkerInitializer(
    const std::string& camera_frame_id,
    const std::string& object_package,
    const std::string& object_directory,
    const std::vector<std::string>& object_names,
    const std::vector<geometry_msgs::Pose>& poses,
    bool load_from_cache,
    bool accept_cached_poses)
    : load_from_cache_(load_from_cache),
      accept_cached_poses_(accept_cached_poses),
      server_(std::make_shared<interactive_markers::InteractiveMarkerServer>(
          "interactive_marker_initializer")),
      camera_frame_id_(camera_frame_id)
{
    if (load_from_cache_ && accept_cached_poses_)
    {
        ROS_INFO(
            "Skipping interactive marker initializer. Using cached poses.");
        poses_.resize(object_names.size());
        for (size_t i = 0; i < object_names.size(); ++i)
        {
            std::string name = std::to_string(i);
            load_pose_from_cache(name, poses_[i]);
        }
    }
    else
    {
        set_objects(object_package,
                    object_directory,
                    object_names,
                    poses,
                    load_from_cache);
    }
}

InteractiveMarkerInitializer::InteractiveMarkerInitializer(
    const std::string& camera_frame_id)
    : server_(std::make_shared<interactive_markers::InteractiveMarkerServer>(
          "interactive_marker_initializer")),
      camera_frame_id_(camera_frame_id)
{
}

void InteractiveMarkerInitializer::set_object(
    const std::string& object_package,
    const std::string& object_directory,
    const std::string& object_name,
    const geometry_msgs::Pose& pose,
    bool load_from_cache,
    bool make_active)
{
    set_objects(object_package,
                object_directory,
                {object_name},
                {pose},
                load_from_cache,
                make_active);
}

void InteractiveMarkerInitializer::set_objects(
    const std::string& object_package,
    const std::string& object_directory,
    const std::vector<std::string>& object_names,
    const std::vector<geometry_msgs::Pose>& poses,
    bool load_from_cache,
    bool make_active)
{
    poses_ = poses;

    server_->clear();
    poses_.resize(object_names.size());

    // activate all markers
    active_.resize(object_names.size());
    for (int i = 0; i < active_.size(); ++i)
    {
        active_[i] = make_active;
    }

    // If poses provided they have to match the number of objects
    if (poses.size() > 0 && object_names.size() != poses.size())
    {
        ROS_ERROR("#object names != #poses");
        exit(1);
    }


    for (size_t i = 0; i < object_names.size(); i++)
    {
        // create an interactive marker for our server
        visualization_msgs::InteractiveMarker int_marker;
        std::string name = std::to_string(i);
        create_interactive_marker(
            camera_frame_id_,
            name,
            "[active] Click on object when aligned to initialize",
            int_marker);

        if (poses.size() > 0)
        {
            int_marker.pose = poses[i];
        }

        poses_[i] = int_marker.pose;

        add_object_controller(
            object_package, object_directory, object_names[i], int_marker);
        add_controllers(int_marker);

        switch_marker(int_marker, make_active);

        // add the interactive marker to our collection &
        // tell the server to call process_feedback() when feedback arrives
        // for
        // it
        server_->insert(
            int_marker,
            boost::bind(
                &InteractiveMarkerInitializer::process_feedback, this, _1));
    }

    // 'commit' changes and send to all clients
    server_->applyChanges();
}

void InteractiveMarkerInitializer::poses_update_callback(Callback callback)
{
    poses_update_callback_ = callback;
}

void InteractiveMarkerInitializer::delete_poses_update_callback()
{
    poses_update_callback_ = Callback();
}

bool InteractiveMarkerInitializer::are_all_object_poses_set(
    bool accept_cached_poses)
{
    if (accept_cached_poses && load_from_cache_)
    {
        return true;
    }

    for (bool still_active : active_)
    {
        if (still_active) return false;
    }

    return true;
}

bool InteractiveMarkerInitializer::wait_for_object_poses()
{
    ROS_INFO(
        "Please use rviz to align and initialize the object poses under "
        "the "
        "topic.");
    ROS_INFO("Waiting for all interactive object poses to be set ...");
    while (!are_all_object_poses_set(accept_cached_poses_))
    {
        if (!ros::ok()) return false;

        ros::Duration(1.e-3).sleep();
        ros::spinOnce();
    }

    return true;
}

const std::vector<geometry_msgs::Pose>& InteractiveMarkerInitializer::poses()
{
    return poses_;
}

const geometry_msgs::PoseArray InteractiveMarkerInitializer::pose_array()
{
    geometry_msgs::PoseArray pose_array_msg;
    pose_array_msg.header.frame_id = camera_frame_id_;
    pose_array_msg.header.stamp    = ros::Time::now();
    pose_array_msg.poses           = poses();

    return pose_array_msg;
}

void InteractiveMarkerInitializer::create_interactive_marker(
    const std::string frame_id,
    const std::string name,
    const std::string description,
    visualization_msgs::InteractiveMarker& int_marker)
{
    int_marker.header.frame_id = frame_id;
    int_marker.header.stamp    = ros::Time::now();
    int_marker.name            = name;
    int_marker.description     = description;
    int_marker.scale           = 0.2;
    // position interactive marker by default 1 m in font
    // of camera in viewing condition
    int_marker.pose.position.x    = 0.0;
    int_marker.pose.position.y    = 0.0;
    int_marker.pose.position.z    = 1.0;
    int_marker.pose.orientation.w = 1;
    int_marker.pose.orientation.x = 0;
    int_marker.pose.orientation.y = 0;
    int_marker.pose.orientation.z = 0;

    // load pose from cache if available
    if (load_from_cache_)
    {
        load_pose_from_cache(name, int_marker.pose);
    }
}

void InteractiveMarkerInitializer::add_object_controller(
    const std::string& object_package,
    const std::string& object_directory,
    const std::string& object_name,
    visualization_msgs::InteractiveMarker& int_marker)
{
    boost::filesystem::path object_model_path("package://");
    object_model_path /= object_package;
    object_model_path /= object_directory;
    object_model_path /= object_name;

    ROS_INFO("Creating marker: %s", object_model_path.string().c_str());

    // create a grey box marker
    visualization_msgs::Marker object_marker;
    object_marker.type          = visualization_msgs::Marker::MESH_RESOURCE;
    object_marker.mesh_resource = object_model_path.string();
    object_marker.scale.x       = 1.0;
    object_marker.scale.y       = 1.0;
    object_marker.scale.z       = 1.0;
    object_marker.color.r       = 0.5;
    object_marker.color.g       = 0.5;
    object_marker.color.b       = 0.5;
    object_marker.color.a       = 1.0;
    object_marker.pose.orientation.w = 1;
    object_marker.pose.orientation.x = 0;
    object_marker.pose.orientation.y = 0;
    object_marker.pose.orientation.z = 0;

    // create a non-interactive control which contains the object
    visualization_msgs::InteractiveMarkerControl object_control;
    object_control.interaction_mode =
        visualization_msgs::InteractiveMarkerControl::BUTTON;
    object_control.name           = "button_control";
    object_control.always_visible = true;
    object_control.orientation.w  = 1;
    object_control.markers.push_back(object_marker);

    // add the control to the interactive marker
    int_marker.controls.push_back(object_control);
}

void InteractiveMarkerInitializer::add_controllers(
    visualization_msgs::InteractiveMarker& int_marker)
{
    visualization_msgs::InteractiveMarkerControl control;
    control.orientation.w = 1;
    control.orientation.x = 1;
    control.orientation.y = 0;
    control.orientation.z = 0;
    control.name          = "rotate_x";
    control.interaction_mode =
        visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_x";
    control.interaction_mode =
        visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 1;
    control.orientation.z = 0;
    control.name          = "rotate_z";
    control.interaction_mode =
        visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_z";
    control.interaction_mode =
        visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 0;
    control.orientation.z = 1;
    control.name          = "rotate_y";
    control.interaction_mode =
        visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_y";
    control.interaction_mode =
        visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);
}

void InteractiveMarkerInitializer::process_feedback(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
{
    std::lock_guard<std::mutex> lock(mutex_);

    // update marker pose
    int i = std::stoi(feedback->marker_name);

    ROS_INFO("Marker Frame %s", feedback->header.frame_id.c_str());

    // if the feedback from the marker is in a different than the desired
    // frame
    // convert it using a transform listener
    if (feedback->header.frame_id.compare(camera_frame_id_) != 0)
    {
        geometry_msgs::PoseStamped pose_in;
        pose_in.header = feedback->header;
        pose_in.pose   = feedback->pose;
        geometry_msgs::PoseStamped pose_out;
        try
        {
            listener_.transformPose(camera_frame_id_, pose_in, pose_out);
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("%s", ex.what());
            ros::Duration(1.0).sleep();
        }

        poses_[i] = pose_out.pose;
    }
    else
    {
        poses_[i] = feedback->pose;
    }

    visualization_msgs::InteractiveMarker int_marker;
    server_->get(feedback->marker_name, int_marker);
    if (feedback->event_type ==
        visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK)
    {
        ROS_INFO("Feedback frame %s and Marker Frame %s",
                 feedback->header.frame_id.c_str(),
                 int_marker.header.frame_id.c_str());

        // cache this pose
        cache_pose(feedback->marker_name, poses_[i]);

        // toggle active state
        active_[i] = !active_[i];

        switch_marker(int_marker, active_[i]);

        if (are_all_object_poses_set() && poses_update_callback_)
        {
            poses_update_callback_(pose_array());
        }

        // update marker

        int_marker.header.frame_id = feedback->header.frame_id;

        int_marker.controls.front().orientation.w = 1;
        int_marker.pose.orientation.w             = 1;
        int_marker.pose.orientation.x             = 0;
        int_marker.pose.orientation.y             = 0;
        int_marker.pose.orientation.z             = 0;

        // apply no changes this will cause rviz to crash (with ogre 1.9)
        // server_->insert(int_marker);
        // server_->applyChanges();
        // same happens when destroying the marker server
        // server_.reset();
    }
}

void InteractiveMarkerInitializer::switch_marker(
    visualization_msgs::InteractiveMarker& int_marker, bool active)
{
    if (active)
    {
        int_marker.description =
            "[active] Click on object when aligned to initialize";
        add_controllers(int_marker);
        int_marker.controls.front().markers.front().color.a = 1.0;
    }
    else
    {
        int_marker.description = "[inactive] Click to active";
        int_marker.controls.erase(int_marker.controls.begin() + 1,
                                  int_marker.controls.end());
        int_marker.controls.front().markers.front().color.a = 0.2;
    }
}

void InteractiveMarkerInitializer::load_pose_from_cache(
    const std::string& object_name, geometry_msgs::Pose& pose) const
{
    std::ifstream pose_tmp_file;
    std::string cache_file = "/tmp/pose_cache_";
    cache_file += object_name + ".txt";
    pose_tmp_file.open(cache_file.c_str());
    if (pose_tmp_file.is_open())
    {
        ROS_INFO("Loading interactive marker pose %s from ",
                 cache_file.c_str());


        pose_tmp_file >> pose.position.x;
        pose_tmp_file >> pose.position.y;
        pose_tmp_file >> pose.position.z;
        pose_tmp_file >> pose.orientation.w;
        pose_tmp_file >> pose.orientation.x;
        pose_tmp_file >> pose.orientation.y;
        pose_tmp_file >> pose.orientation.z;
        pose_tmp_file.close();
    }
    else
    {
        ROS_WARN_STREAM("No cached pose for object " << object_name);
    }

    double length = std::sqrt(
        std::pow(pose.orientation.w, 2) + std::pow(pose.orientation.x, 2) +
        std::pow(pose.orientation.y, 2) + std::pow(pose.orientation.z, 2));
    pose.orientation.w /= length;
    pose.orientation.x /= length;
    pose.orientation.y /= length;
    pose.orientation.z /= length;
}

void InteractiveMarkerInitializer::cache_pose(
    const std::string& object_name, const geometry_msgs::Pose& pose) const
{
    std::ofstream pose_tmp_file;
    std::string cache_file = "/tmp/pose_cache_";
    cache_file += object_name + ".txt";
    ROS_INFO("Caching interactive marker pose in %s", cache_file.c_str());

    pose_tmp_file.open(cache_file.c_str());
    pose_tmp_file << pose.position.x << " " << pose.position.y << " "
                  << pose.position.z << " ";
    pose_tmp_file << pose.orientation.w << " " << pose.orientation.x << " "
                  << pose.orientation.y << " " << pose.orientation.z;
    pose_tmp_file.close();
}
}
