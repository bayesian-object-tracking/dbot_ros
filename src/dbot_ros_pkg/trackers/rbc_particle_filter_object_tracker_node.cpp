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


#include <Eigen/Dense>

#include <fstream>
#include <ctime>

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>

#include <boost/filesystem.hpp>

#include <fl/util/profiling.hpp>

#include <dbot_ros_pkg/trackers/rbc_particle_filter_object_tracker.hpp>
#include <dbot_ros_pkg/utils/pcl_interface.hpp>
#include <dbot_ros_pkg/utils/ros_interface.hpp>

#include <opi/interactive_marker_initializer.hpp>
#include <osr/free_floating_rigid_bodies_state.hpp>

// #include <dbot_ros_pkg/utils/interactive_marker_wrapper.hpp>

typedef sensor_msgs::CameraInfo::ConstPtr CameraInfoPtr;
typedef Eigen::Matrix<double, -1, -1> Image;

using namespace bot;

class Tracker
{
public:
    Tracker(boost::shared_ptr<RbcParticleFilterObjectTracker> tracker): tracker_(tracker), node_handle_("~")
    {
        std::string config_file;
        ri::ReadParameter("config_file", config_file, node_handle_);

        path_ = config_file;
        path_ = path_.parent_path();
        std::cout << path_ << std::endl;

        time_t rawtime;
        struct tm * timeinfo;
        char buffer[80];

        time (&rawtime);
        timeinfo = localtime(&rawtime);

        strftime(buffer,80,"%d.%m.%Y_%I.%M.%S",timeinfo);
        std::string current_time(buffer);

        path_ /= "tracking_data_" + current_time + ".txt";
    }
    ~Tracker() {}

    void Filter(const sensor_msgs::Image& ros_image)
    {
        INIT_PROFILING
        osr::FreeFloatingRigidBodiesState<-1> mean_state = tracker_->Filter(ros_image);
        MEASURE("total time for filtering")
    }

    void FilterAndStore(const sensor_msgs::Image& ros_image)
    {
        INIT_PROFILING
        osr::FreeFloatingRigidBodiesState<-1> mean_state = tracker_->Filter(ros_image);
        MEASURE("total time for filtering")

        std::ofstream file;
        file.open(path_.c_str(), std::ios::out | std::ios::app);
        if(file.is_open())
        {
            file << ros_image.header.stamp << " ";
            file << mean_state.poses().transpose() << std::endl;
            file.close();
        }
        else
        {
            std::cout << "could not open file " << path_ << std::endl;
            exit(-1);
        }
    }

private:
    boost::shared_ptr<RbcParticleFilterObjectTracker> tracker_;
    ros::NodeHandle node_handle_;
    boost::filesystem::path path_;
};

int main (int argc, char **argv)
{
    ros::init(argc, argv, "test_filter");
    ros::NodeHandle node_handle("~");

    // read parameters
    std::cout << "reading parameters" << std::endl;
    std::string depth_image_topic; ri::ReadParameter("depth_image_topic", depth_image_topic, node_handle);
    std::string camera_info_topic; ri::ReadParameter("camera_info_topic", camera_info_topic, node_handle);
    double min_delta_time; ri::ReadParameter("min_delta_time", min_delta_time, node_handle);
    std::string source; ri::ReadParameter("source", source, node_handle);
//    std::vector<std::string> object_names; ri::ReadParameter("object_names", object_names, node_handle);

    int initial_sample_count; ri::ReadParameter("initial_sample_count", initial_sample_count, node_handle);

    std::cout << "reading data from camera " << std::endl;
    Eigen::Matrix3d camera_matrix = ri::GetCameraMatrix<double>(camera_info_topic, node_handle, 2.0);
//    std::string frame_id = ri::GetCameraFrame<double>(camera_info_topic, node_handle, 2.0);

        

    std::string camera_frame_id;
    std::string object_package;
    std::string object_directory;
    std::vector<std::string> object_names;

    node_handle.getParam("object_names", object_names);
    node_handle.getParam("object_package", object_package);
    node_handle.getParam("object_directory", object_directory);
    node_handle.getParam("camera_frame_id", camera_frame_id);

    opi::InteractiveMarkerInitializer im_server(
        camera_frame_id, object_package, object_directory, object_names);

    while(!im_server.all_object_poses_set() && ros::ok())
    {
        ros::Duration(1.e-3).sleep();
        ros::spinOnce();
    }

    auto initial_poses = im_server.poses();
    std::vector<Eigen::VectorXd> initial_states;

//        std::vector<Eigen::VectorXd> initial_states = im_server.getMarkerPose();

    for (int i = 0; i < initial_poses.size(); ++i)
    {
        std::cout << "tracking object: " << object_names[i] << std::endl;

        Eigen::Vector3d p;
        Eigen::Quaternion<double> q;
        p[0] = initial_poses[0].position.x;
        p[1] = initial_poses[0].position.y;
        p[2] = initial_poses[0].position.z;
        q.w() = initial_poses[0].orientation.w;
        q.x() = initial_poses[0].orientation.x;
        q.y() = initial_poses[0].orientation.y;
        q.z() = initial_poses[0].orientation.z;
        osr::PoseVelocityVector pose;
        pose.position() = p;
        pose.orientation().quaternion(q);
        initial_states.push_back(pose);
    }

    std::cout << "Number of object poses: " << initial_states.size() << std::endl;
    std::cout << initial_states[0] << std::endl;
    
//    std::vector<Eigen::VectorXd> initial_states =im_server.getMarkerPose();
      
    // get observations from camera
    sensor_msgs::Image::ConstPtr ros_image =
            ros::topic::waitForMessage<sensor_msgs::Image>(depth_image_topic, node_handle, ros::Duration(10.0));

    // intialize the filter
    boost::shared_ptr<RbcParticleFilterObjectTracker> tracker(new RbcParticleFilterObjectTracker);
    tracker->Initialize(initial_states, *ros_image, camera_matrix);
    std::cout << "done initializing" << std::endl;
    Tracker interface(tracker);

    ros::Subscriber subscriber = node_handle.subscribe(depth_image_topic, 1, &Tracker::Filter, &interface);

    ros::spin();

//    while (ros::ok())
//      {
//	// in case interactive marker gets clicked again, re-initialize the filter
////	if(im_server.initializeObjects())
////	  {
//	    // get current observations from camera
////	    ros_image = ros::topic::waitForMessage<sensor_msgs::Image>(depth_image_topic, node_handle, ros::Duration(10.0));
//	    // get desired initialization pose
////	    initial_states =im_server.getMarkerPose();
////	    tracker->Initialize(initial_states, *ros_image, camera_matrix);
////	  }
//        ros::spinOnce();
//      }

    return 0;
}
