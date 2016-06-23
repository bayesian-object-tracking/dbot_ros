/**
 * \file rmsgf_object_tracker_node.cpp
 * \date September 2015
 * \author Jan Issac (jan.issac@gmail.com)
 */


#include <fstream>
#include <ctime>
#include <memory>
#include <mutex>

#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Header.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>

#include <boost/filesystem.hpp>

#include <fl/util/types.hpp>
#include <fl/util/meta.hpp>
#include <fl/util/profiling.hpp>
#include <fl/model/observation/linear_cauchy_observation_model.hpp>

#include <dbot/common/rigid_body_renderer.hpp>
#include <dbot/common/helper_functions.hpp>

#include <dbot_ros_pkg/utils/ros_interface.hpp>
#include <dbot_ros_pkg/utils/object_file_reader.hpp>

#include <opi/interactive_marker_initializer.hpp>
#include <osr/free_floating_rigid_bodies_state.hpp>

#include <dbot_ros_pkg/trackers/gaussian/rmsgf_object_tracker.hpp>

using namespace fl;
using namespace rmsgf;

class RmsgfObjectTrackerNode
{
public:
    /* ------------------------------ */
    /* - The Filter                 - */
    /* ------------------------------ */
    typedef RmsgfObjectTracker<fl::RobustMultiSensorGaussianFilter,
                               fl::UniformObservationModel,
                               fl::SigmaPointQuadrature<fl::UnscentedTransform>,
                               osr::PoseVelocityVector> Tracker;

    typedef typename Tracker::State State;

    struct Parameter
    {
        Parameter(Args args) : tracker_param(args)
        {
            ros::NodeHandle nh("~");

            args.get("step_through", step_through);

            args.get("gt_color_r", gt_color_r);
            args.get("gt_color_g", gt_color_g);
            args.get("gt_color_b", gt_color_b);

            args.get("belief_color_r", belief_color_r);
            args.get("belief_color_g", belief_color_g);
            args.get("belief_color_b", belief_color_b);

            // get object name (file name without extension)

            std::string object_package;
            std::string object_directory;

            std::vector<std::string> object_names;
            nh.getParam("object_names", object_names);

            std::string object_name = object_names[0];
            args.get("object_package", object_package);
            args.get("object_directory", object_directory);

            // get model resource URI,
            // i.e. package://[package name]/.../[obj name].obj
            boost::filesystem::path object_model_uri_path("package://");
            object_model_uri_path /= object_package;
            object_model_uri_path /= object_directory;
            object_model_uri_path /= object_name + ".obj";
            object_model_uri = object_model_uri_path.string();

            // get model resource absolute path, i.e. /home/.../[obj name].obj
            boost::filesystem::path object_model_absolute_path(
                ros::package::getPath(object_package));
            object_model_absolute_path /= object_directory;
            object_model_absolute_path /= object_name + ".obj";
            ;
            object_model_path = object_model_absolute_path.string();

            args.get("downsampling", downsampling);
            args.get("resX", resX);
            args.get("resY", resY);
            width = resX / downsampling;
            height = resY / downsampling;
        }

        bool initialize_from_cache;

        int downsampling;
        bool step_through;

        int gt_color_r;
        int gt_color_g;
        int gt_color_b;

        int belief_color_r;
        int belief_color_g;
        int belief_color_b;

        int height;
        int width;

        int resX;
        int resY;

        std::string object_model_uri;
        std::string object_model_path;

        typename Tracker::Parameter tracker_param;

        void print()
        {
            PF(height);
            PF(width);
            PF(step_through);

            PF(downsampling);
            PF(resX);
            PF(resY);

            tracker_param.print();
        }
    };

    typedef std::shared_ptr<Tracker> TrackerPtr;
    typedef std::shared_ptr<dbot::RigidBodyRenderer> RigidBodyRendererPtr;

public:
    RmsgfObjectTrackerNode(ros::NodeHandle& nh)
        : param(Args(nh)),
          last_measurement_time_(std::numeric_limits<Real>::quiet_NaN())

    {
        image.setZero(1, 1);
        param.print();

        /* ------------------------------ */
        /* - Setup camera               - */
        /* ------------------------------ */
        std::string depth_image_topic;
        ri::ReadParameter("depth_image_topic", depth_image_topic, nh);

        std::string camera_info_topic;
        ri::ReadParameter("camera_info_topic", camera_info_topic, nh);

        std::cout << "reading data from camera " << std::endl;
        camera_matrix = ri::GetCameraMatrix<double>(camera_info_topic, nh, 2.0);
        camera_matrix(0, 0) /= param.downsampling;  // fx
        camera_matrix(1, 1) /= param.downsampling;  // fy
        camera_matrix(2, 2) = 1.0;
        camera_matrix(0, 2) /= param.downsampling;  // cx
        camera_matrix(1, 2) /= param.downsampling;  // cy

        ri::ReadParameter("camera_frame_id", camera_frame_id, nh);

        std::string camera_frame_id;
        std::string object_package;
        std::string object_directory;
        std::vector<std::string> object_names;

        nh.getParam("object_names", object_names);
        nh.getParam("object_package", object_package);
        nh.getParam("object_directory", object_directory);
        nh.getParam("camera_frame_id", camera_frame_id);

        opi::InteractiveMarkerInitializer im_server(
            camera_frame_id, object_package, object_directory, object_names);

        while (!im_server.are_all_object_poses_set() && ros::ok())
        {
            ros::Duration(1.e-3).sleep();
            ros::spinOnce();
        }

        auto initial_states = im_server.poses();

        //        std::vector<Eigen::VectorXd> initial_states =
        //        im_server.getMarkerPose();

        Eigen::Vector3d p;
        Eigen::Quaternion<double> q;
        p[0] = initial_states[0].position.x;
        p[1] = initial_states[0].position.y;
        p[2] = initial_states[0].position.z;
        q.w() = initial_states[0].orientation.w;
        q.x() = initial_states[0].orientation.x;
        q.y() = initial_states[0].orientation.y;
        q.z() = initial_states[0].orientation.z;
        pose.position() = p;
        pose.orientation().quaternion(q);

        object_publisher =
            nh.advertise<visualization_msgs::Marker>("object_model", 0);

        subscriber = nh.subscribe(depth_image_topic,
                                  1,
                                  &RmsgfObjectTrackerNode::update_observation,
                                  this);

        tracker = std::shared_ptr<Tracker>(
            new Tracker(pose,
                        create_object_renderer(param, camera_matrix),
                        param.tracker_param));
    }

    void update_observation(const sensor_msgs::Image& ros_image)
    {
        std::lock_guard<std::mutex> lock(mutex);
        image = ri::Ros2EigenVector<double>(ros_image, param.downsampling);

        header = ros_image.header;
    }

    void track()
    {
        if (image.size() == 1)
        {
            ros::Duration(1. / 60.).sleep();
            return;
        }

        //        std::lock_guard<std::mutex> lock(mutex);

        //        if(std::isnan(last_measurement_time_))
        //            last_measurement_time_ = ros_image.header.stamp.toSec();
        //        Real delta_time =
        //                ros_image.header.stamp.toSec() -
        //                last_measurement_time_;
        //        last_measurement_time_ = ros_image.header.stamp.toSec();
        //        std::cout << "actual delta time " << delta_time << std::endl;
        Eigen::VectorXd y;
        {
            std::lock_guard<std::mutex> lock(mutex);
            y = image;
        }

        INIT_PROFILING
        tracker->filter_once(y);
        MEASURE("tracker->filter(image)");

        PVT(tracker->belief().mean());
        PV(tracker->belief().covariance());

        header.stamp = ros::Time::now();

        ri::PublishMarker(tracker->belief().mean().homogeneous().cast<float>(),
                          header,
                          param.object_model_uri,
                          object_publisher,
                          1,
                          param.belief_color_r / 255.,
                          param.belief_color_g / 255.,
                          param.belief_color_b / 255.,
                          1.0,
                          "object");
    }

public: /* Factory Functions  */
    RigidBodyRendererPtr create_object_renderer(Parameter& params,
                                                Eigen::Matrix3d& camera_matrix)
    {
        std::cout << "Opening object file " << params.object_model_path
                  << std::endl;
        std::vector<Eigen::Vector3d> object_vertices;
        std::vector<std::vector<int>> object_triangle_indices;
        ObjectFileReader file_reader;
        file_reader.set_filename(params.object_model_path);
        file_reader.Read();
        object_vertices = *file_reader.get_vertices();
        object_triangle_indices = *file_reader.get_indices();

        RigidBodyRendererPtr object_renderer(
            new dbot::RigidBodyRenderer({object_vertices},
                                        {object_triangle_indices},
                                        camera_matrix,
                                        param.height,
                                        param.width));

        return object_renderer;
    }

protected:
    Parameter param;
    Eigen::Matrix3d camera_matrix;
    State pose;

private:
    // Eigen::Matrix<Eigen::Vector3d, -1, -1> points;
    Eigen::VectorXd image;
    ros::Publisher object_publisher;
    ros::Subscriber subscriber;
    std::string camera_frame_id;
    std::shared_ptr<Tracker> tracker;
    std::mutex mutex;

    Real last_measurement_time_;

    std_msgs::Header header;
};

int main(int argc, char** argv)
{
    /* ------------------------------ */
    /* - Setup ros                  - */
    /* ------------------------------ */
    ros::init(argc, argv, "rmsgf_object_tracker");
    ros::NodeHandle nh("~");

    RmsgfObjectTrackerNode node(nh);

    while (ros::ok())
    {
        node.track();
        ros::spinOnce();
    }

    return 0;
}
