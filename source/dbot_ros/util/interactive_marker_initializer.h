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
 * \file interactive_markers_initializer.h
 * \date October 2015
 * \author Jeannette Bohg (bohg.jeannette@googlemail.com)
 * \author Jan Issac (jan.issac@gmail.com)
 */

#pragma once

#include <Eigen/Dense>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <interactive_markers/interactive_marker_server.h>
#include <memory>
#include <mutex>
#include <ros/ros.h>
#include <tf/transform_listener.h>

namespace opi
{
class InteractiveMarkerInitializer
{
public:
    typedef std::function<void(const geometry_msgs::PoseArray&)> Callback;

public:
    /**
     * \brief Creates an Interactive marker initializer with the desired object
     */
    InteractiveMarkerInitializer(const std::string& camera_frame_id,
                                 const std::string& objec_package,
                                 const std::string& object_directory,
                                 const std::vector<std::string>& object_names,
                                 const std::vector<geometry_msgs::Pose>& poses,
                                 bool load_from_cache,
                                 bool accept_cached_poses = false);

    /**
     *
     * \brief Creates an interactive marker initializer with no object added
     */
    InteractiveMarkerInitializer(const std::string& camera_frame_id);

    /**
     * \brief Default virtual constructor
     */
    virtual ~InteractiveMarkerInitializer() {}
    /**
     * \brief Sets the interactive marker for the desired object
     */
    void set_objects(const std::string& object_package,
                     const std::string& object_directory,
                     const std::vector<std::string>& object_names,
                     const std::vector<geometry_msgs::Pose>& poses,
                     bool load_from_cache,
                     bool make_active = true);

    /**
     * \brief Sets the interactive marker for the desired object
     */
    void set_object(const std::string& object_package,
                    const std::string& object_directory,
                    const std::string& object_name,
                    const geometry_msgs::Pose& pose,
                    bool load_from_cache,
                    bool make_active = true);

    /**
     * \brief Returns a vector of poses provided by the interactive markers
     * objects
     */
    const std::vector<geometry_msgs::Pose>& poses();

    /**
     * \brief Returns a pose array message provided by the interactive markers
     * objects
     */
    const geometry_msgs::PoseArray pose_array();

    /**
     * \brief Checks wheather all object poses have been aligned
     * \param accept_cached_poses If set to true and cached poses where loaded,
     *        this will return true.
     */
    bool are_all_object_poses_set(bool accept_cached_poses = false);

    /**
     * \brief A call to this fucntion will block until all object poses have
     *        been set.
     * \return True if all objects have been set and returns false if
     *         interrupted by a signal.
     */
    bool wait_for_object_poses();

    /**
     * \brief Set the callback function for object poses update
     *
     * The callback function is called when all objects have been aligned
     */
    void poses_update_callback(Callback callback);

    void switch_marker(visualization_msgs::InteractiveMarker& int_marker,
                       bool active);

    void delete_poses_update_callback();

protected:
    /**
     * \brief Creates a new interactive marker
     */
    void create_interactive_marker(
        const std::string frame_id,
        const std::string name,
        const std::string description,
        visualization_msgs::InteractiveMarker& int_marker);

    /**
     * \brief Addes the object model as a button to the interactive marker.
     * The object model mesh is loaded from the URI
     * package://object_package/object_directory/object_name.obj
     */
    void add_object_controller(
        const std::string& object_package,
        const std::string& object_directory,
        const std::string& object_name,
        visualization_msgs::InteractiveMarker& int_marker);

    /**
     * \brief Addes 6-DoF controllers to the given marker
     *
     */
    void add_controllers(visualization_msgs::InteractiveMarker& int_marker);

    /**
     * \brief Removes the 6-DoF controllers from the given marker
     */
    void remove_controllers(visualization_msgs::InteractiveMarker& int_marker);

    /**
     * \brief Process marker interaction feedback
     */
    void process_feedback(
        const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);

    /**
     * \brief Writes a pose for the object \c object_name to its cache file
     */
    void cache_pose(const std::string& object_name,
                    const geometry_msgs::Pose& pose) const;

    /**
     * \brief Loads the pose of the object specified by \c object_name from its
     * cache
     * file
     */
    void load_pose_from_cache(const std::string& object_name,
                              geometry_msgs::Pose& pose) const;

protected:
    /**
     * \brief Object status vector. The status is active if the object has not
     * been aligned yet. The object is set inactive (aligned) after clicking on
     * it.
     */
    std::vector<bool> active_;

    bool load_from_cache_;
    bool accept_cached_poses_;

    /**
     * \brief Poses container of all interactive markers
     */
    std::vector<geometry_msgs::Pose> poses_;

    /**
     * \brief Callback function which is called when all object have be aligned
     * and set
     */
    Callback poses_update_callback_;

private:
    /**
     * \brief Marker callback mutex
     */
    std::mutex mutex_;

    /**
     * \brief Interactive marker backend
     */
    std::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;

    /**
     * \brief Camera reference frame for the markers
     */
    std::string camera_frame_id_;

    /**
     * \brief Transform listener to convert feedback from interactive marker
     * into desired frame.
     */
    tf::TransformListener listener_;
};
}
