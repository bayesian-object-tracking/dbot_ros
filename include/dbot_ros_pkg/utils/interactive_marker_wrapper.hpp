/*************************************************************************
This software allows for filtering in high-dimensional observation and
state spaces, as described in

M. Wuthrich, P. Pastor, M. Kalakrishnan, J. Bohg, and S. Schaal.
Probabilistic Object Tracking using a Range Camera
IEEE/RSJ Intl Conf on Intelligent Robots and Systems, 2013

In a publication based on this software pleace cite the above reference.


Copyright (C) 2015  Jeannette Bohg

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

#ifndef POSE_TRACKING_INTERFACE_UTILS_INTERACTIVE_MARKER_HPP
#define POSE_TRACKING_INTERFACE_UTILS_INTERACTIVE_MARKER_HPP

#include <ros/ros.h>
#include <interactive_markers/interactive_marker_server.h>

#include <Eigen/Core>

namespace im
{
  class InteractiveMarkerWrapper
  {
  public:
    InteractiveMarkerWrapper();
    ~InteractiveMarkerWrapper() {};

    std::vector<Eigen::VectorXd> getMarkerPose();
    bool finalPose();
    
  private:

    ros::NodeHandle nh_;
    boost::mutex mutex_;
    
    interactive_markers::InteractiveMarkerServer server_;
    visualization_msgs::InteractiveMarker interactive_marker_;

    void createInteractiveMarker(const std::string frame_id,
				 const std::string name,
				 const std::string description,
				 visualization_msgs::InteractiveMarker &int_marker);
    void addObjectControl(const std::string object_name,
			  visualization_msgs::InteractiveMarker &int_marker);
    void add6DoFControl(visualization_msgs::InteractiveMarker &int_marker);
    
    void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );
    
    std::vector<Eigen::Vector3d> marker_positions_;
    std::vector<Eigen::Quaternion<double> > marker_orientations_;

    std::vector<std::string> object_names_;
    int n_objects_;

    bool button_clicked_;
    
  };

}



#endif
