/**
 * \file rmsgf_object_tracker_simulation_node.hpp
 * \date September 2015
 * \author Jan Issac (jan.issac@gmail.com)
 */

#define PROFILING_ON

#include <fstream>
#include <ctime>
#include <memory>

#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Header.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>

#include <fl/util/types.hpp>
#include <fl/util/meta.hpp>
#include <fl/util/profiling.hpp>

#include <dbot/utils/rigid_body_renderer.hpp>
#include <dbot/utils/helper_functions.hpp>

#include <state_filtering/utils/image_publisher.hpp>
#include <state_filtering/utils/ros_interface.hpp>
#include <state_filtering/utils/pcl_interface.hpp>

#include <fl/model/observation/linear_cauchy_observation_model.hpp>

#include "rmsgf_tracking_ros_pkg/util/virtual_object.hpp"
#include "rmsgf_tracking_ros_pkg/tracker/rmsgf_object_tracker.hpp"

using namespace fl;
using namespace rmsgf;

int main (int argc, char **argv)
{
    /* ------------------------------ */
    /* - Setup ros                  - */
    /* ------------------------------ */
    ros::init(argc, argv, "rmsgf_object_tracker_simulation");
    ros::NodeHandle nh("~");

    /* ------------------------------ */
    /* - The Filter                 - */
    /* ------------------------------ */
    typedef RmsgfObjectTracker<
        /* Filter */
        fl::RobustMultiSensorGaussianFilter,
        /* Tail model */
//        fl::LinearCauchyObservationModel,
        fl::UniformObservationModel,
        /* Quadtrature */
        fl::SigmaPointQuadrature<fl::UnscentedTransform>,
//        fl::SigmaPointQuadrature<fl::MonteCarloTransform<fl::ConstantPointCountPolicy<1000>>>,
//        fl::SigmaPointQuadrature<fl::MonteCarloTransform<fl::LinearPointCountPolicy<15>>>,
        fl::PoseVector
    > Tracker;

    typedef typename Tracker::State State;

    /* ------------------------------ */
    /* - Parse arguments            - */
    /* ------------------------------ */
    Args args(nh);
    struct Parameter
    {
        Parameter(Args& args)
            : tracker_param(args)
        {
            args.get("step_through",           step_through);
            args.get("default_sensor_bg",      default_sensor_bg);

            args.get("error_type",             error_type);
            args.get("error_start_iteration",  error_start_iteration);
            args.get("manual_error_pixels",    manual_error_pixels);
            args.get("manual_error_start",     manual_error_start);
            args.get("manual_error_magnitude", manual_error_magnitude);

            args.get("gt_color_r", gt_color_r);
            args.get("gt_color_g", gt_color_g);
            args.get("gt_color_b", gt_color_b);

            args.get("belief_color_r", belief_color_r);
            args.get("belief_color_g", belief_color_g);
            args.get("belief_color_b", belief_color_b);

            int downsampling;
            args.get("downsampling", downsampling);

            height = 480 / downsampling;
            width = 640 / downsampling;
        }

        bool step_through;
        int error_type;
        int manual_error_pixels;
        int manual_error_start;
        int error_start_iteration;
        double manual_error_magnitude;

        double default_sensor_bg;

        int gt_color_r;
        int gt_color_g;
        int gt_color_b;

        int belief_color_r;
        int belief_color_g;
        int belief_color_b;

        int height;
        int width;

        typename Tracker::Parameter tracker_param;

        void print()
        {
            PF(height);
            PF(width);
            PF(step_through);
            PF(default_sensor_bg);
            PF(error_type);
            PF(manual_error_pixels);
            PF(manual_error_start);
            PF(error_start_iteration);
            PF(manual_error_magnitude);

            tracker_param.print();
        }
    };
    Parameter param(args);


    /* ------------------------------ */
    /* - Setup object               - */
    /* ------------------------------ */
    VirtualObject<State> object(nh);


    /* ------------------------------ */
    /* - Create the tracker         - */
    /* ------------------------------ */
    auto tracker = Tracker(object.pose, object.renderer, param.tracker_param);
    auto& obsrv_model = tracker.filter().obsrv_model();


    /* ------------------------------ */
    /* - Print initial state        - */
    /* ------------------------------ */
    param.print();
    PV(tracker.filter().name());
    PVT(tracker.belief().mean());
    PV(tracker.belief().covariance());

    /* ------------------------------ */
    /* - Images                     - */
    /* ------------------------------ */
    dbot::ImagePublisher ip(nh);

    ros::Publisher cloud_publisher =
        nh.advertise<pcl::PointCloud<pcl::PointXYZ> > ("/tracker/depth/points", 0);


    /* ------------------------------ */
    /* - Simulation                 - */
    /* ------------------------------ */
    fl::StandardGaussian<typename Tracker::ObsrvNoise>
        obsrv_noise_normal(obsrv_model.noise_dimension());
    Eigen::VectorXd ground_truth;
    Eigen::VectorXd y;
    auto zero_pose = State();
    zero_pose.setZero();


    /* ------------------------------ */
    /* - Visualize objects and pause- */
    /* ------------------------------ */
    for (int i = 0; i < 1000 && param.step_through; ++i)
    {
        object.publish_marker(
            object.pose,
            1,
            param.gt_color_r/255.,
            param.gt_color_g/255.,
            param.gt_color_b/255.,
            "gt");
        object.publish_marker(
            tracker.belief().mean(),
            2,
            param.belief_color_r/255.,
            param.belief_color_g/255.,
            param.belief_color_b/255.,
            "belief");
        ros::Duration(0.001).sleep();
        ros::spinOnce();
    }


    while(ros::ok())
    {
        int local_manual_error_pixels = param.manual_error_pixels;
        int local_manual_error_start = param.manual_error_start;

        /* ------------------------------ */
        /* - Animate & Render           - */
        /* ------------------------------ */
        object.animate();

        /* ------------------------------ */
        /* - Simulate observation       - */
        /* ------------------------------ */
        object.render(ground_truth);

        for (int i = 0; i < ground_truth.size(); ++i)
        {
            if (!std::isfinite(ground_truth(i)))
            {
                ground_truth(i) = param.default_sensor_bg;
            }
        }

        if (param.error_start_iteration > 0)
        {
            y = ground_truth;
            param.error_start_iteration--;
        }
        else
        {
            switch (param.error_type)
            {
            case 0: // no error
                y = ground_truth;
                break;

            case 1: // simulated error
                Tracker::Builder::set_nominal_pose(tracker.filter(), zero_pose);
                y = obsrv_model.observation(
                        object.pose, obsrv_noise_normal.sample());
                for (int i = 0; i < y.size(); ++i)
                {
                    if (!std::isfinite(y(i)))
                    {
                        y(i) = param.default_sensor_bg;
                    }
                }
                break;

            case 2: // manual error
                y = ground_truth;

                for (int i = 0; i < ground_truth.size(); ++i)
                {
                    if (std::isfinite(ground_truth(i))
                        && ground_truth(i) > 0
                        && local_manual_error_pixels > 0)
                    {
                        if (local_manual_error_start > 0)
                        {
                            --local_manual_error_start;
                        }
                        else
                        {
                            local_manual_error_pixels--;
                            y(i) += param.manual_error_magnitude;
                        }
                    }
                }
                break;
            case 3: // manual + simulated error
                Tracker::Builder::set_nominal_pose(tracker.filter(), zero_pose);
                y = obsrv_model.observation(
                        object.pose, obsrv_noise_normal.sample());
                for (int i = 0; i < y.size(); ++i)
                {
                    if (!std::isfinite(y(i)) && param.default_sensor_bg > 0)
                    {
                        y(i) = param.default_sensor_bg;
                    }
                }
                for (int i = 0; i < ground_truth.size(); ++i)
                {

                    if (std::isfinite(ground_truth(i))
                        && ground_truth(i) > 0
                        && local_manual_error_pixels > 0)
                    {
                        if (local_manual_error_start > 0)
                        {
                            --local_manual_error_start;
                        }
                        else
                        {
                            local_manual_error_pixels--;
                            y(i) = param.manual_error_magnitude;
                        }
                    }
                }
                break;
            default:
                std::cout << "unknown error type" << std::endl;
                return 1;
            }
        }

        /* ------------------------------ */
        /* - Filter                     - */
        /* ------------------------------ */
        INIT_PROFILING
        tracker.filter_once(y);
        MEASURE("tracker.filter_once()");

        PVT(tracker.belief().mean());
        PV(tracker.belief().covariance());

        /* ------------------------------ */
        /* - Visualize objects          - */
        /* ------------------------------ */
        object.publish_marker(
            object.pose,
            1,
            param.gt_color_r/255.,
            param.gt_color_g/255.,
            param.gt_color_b/255.,
            "gt");
        object.publish_marker(
            tracker.belief().mean(),
            2,
            param.belief_color_r/255.,
            param.belief_color_g/255.,
            param.belief_color_b/255.,
            "belief");

        /* ------------------------------ */
        /* - Visualize images           - */
        /* ------------------------------ */
        ip.publish(
            ground_truth, "/tracker/groundtruth", param.height, param.width);

        ip.publish(
            y, "/tracker/obsrv", param.height, param.width);

        ip.publish(
            tracker.filter().mean_obsrv, "/tracker/predicted_obsrv", param.height, param.width);

        ip.publish(
            (y - ground_truth), "/tracker/diff", param.height, param.width);


        Eigen::MatrixXd y2d =
            Eigen::Map<Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>
                (y.data(), object.res_rows, object.res_cols);

        Eigen::Matrix<Eigen::Vector3d , -1, -1>
            points = dbot::hf::Image2Points(y2d, object.camera_matrix);

        pcl::PointCloud<pcl::PointXYZ>::Ptr
            point_cloud(
                pcl::PointCloud<pcl::PointXYZ>::Ptr(
                    new pcl::PointCloud<pcl::PointXYZ>));

        point_cloud->header = object.header;
        pi::Eigen2Pcl(points, *point_cloud);
        cloud_publisher.publish(point_cloud);

//        ip.publish(
//            tracker.filter().low_level_obsrv_nan,
//            "/tracker/yNANs",
//            param.height, param.width);

//        ip.publish(
//            tracker.filter().low_level_obsrv_fg,
//            "/tracker/fg",
//            param.height, param.width);

//        ip.publish(
//            tracker.filter().low_level_obsrv_bg,
//            "/tracker/bg",
//            param.height, param.width);

        ros::spinOnce();

        /* ------------------------------ */
        /* - Pause                      - */
        /* ------------------------------ */
        if (param.step_through) std::cin.get();
    }

    PV(tracker.filter().name());

    return 0;
}
