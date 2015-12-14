/**
 * \file rmsgf_object_dual_lockstep_tracker_simulation_node.hpp
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

#include <fl/util/types.hpp>
#include <fl/util/meta.hpp>
#include <fl/util/profiling.hpp>

#include <dbot/utils/rigid_body_renderer.hpp>

#include <state_filtering/utils/image_publisher.hpp>
#include <state_filtering/utils/ros_interface.hpp>

#include <fl/model/observation/linear_cauchy_observation_model.hpp>

#include "rmsgf_tracking_ros_pkg/util/virtual_object.hpp"
#include "rmsgf_tracking_ros_pkg/tracker/rmsgf_object_tracker.hpp"

using namespace fl;
using namespace rmsgf;

template <typename Tracker>
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

        args.get("gt_color_r2", gt_color_r2);
        args.get("gt_color_g2", gt_color_g2);
        args.get("gt_color_b2", gt_color_b2);

        args.get("belief_color_r2", belief_color_r2);
        args.get("belief_color_g2", belief_color_g2);
        args.get("belief_color_b2", belief_color_b2);

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


    int gt_color_r2;
    int gt_color_g2;
    int gt_color_b2;

    int belief_color_r2;
    int belief_color_g2;
    int belief_color_b2;

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
        fl::MultiSensorGaussianFilter,
        fl::LinearCauchyObservationModel,
        fl::SigmaPointQuadrature<fl::UnscentedTransform>,
        fl::PoseVector
    > Tracker1;

    typedef RmsgfObjectTracker<
        fl::MultiSensorGaussianFilter,
        fl::LinearCauchyObservationModel,
        fl::SigmaPointQuadrature<fl::UnscentedTransform>,
        fl::PoseVelocityVector
    > Tracker2;


    typedef typename Tracker1::State State1;
    typedef typename Tracker2::State State2;

    /* ------------------------------ */
    /* - Parse arguments            - */
    /* ------------------------------ */
    Args args(nh);


    Parameter<Tracker1> param1(args);
    Parameter<Tracker2> param2(args);


    /* ------------------------------ */
    /* - Setup object               - */
    /* ------------------------------ */
    VirtualObject<State1> object1(nh, "object1");
    VirtualObject<State2> object2(nh, "object2");


    /* ------------------------------ */
    /* - Create the tracker         - */
    /* ------------------------------ */
    auto tracker1 = Tracker1(object1.pose, object1.renderer, param1.tracker_param);
    auto tracker2 = Tracker2(object2.pose, object2.renderer, param2.tracker_param);

    /* ------------------------------ */
    /* - Print initial state        - */
    /* ------------------------------ */

    PVT(tracker1.belief().mean());
    PVT(tracker2.belief().mean());

    PV(tracker1.belief().covariance());
    PV(tracker2.belief().covariance());

    /* ------------------------------ */
    /* - Images                     - */
    /* ------------------------------ */
    dbot::ImagePublisher ip(nh);


    /* ------------------------------ */
    /* - Simulation                 - */
    /* ------------------------------ */
    Eigen::VectorXd ground_truth;
//    Eigen::VectorXd ground_truth;
    Eigen::VectorXd y1;
//    Eigen::VectorXd y1;

    /* ------------------------------ */
    /* - Visualize objects and pause- */
    /* ------------------------------ */
    for (int i = 0; i < 1000 && param1.step_through; ++i)
    {
        object1.publish_marker(
            object1.pose,
            1,
            param1.gt_color_r/255.,
            param1.gt_color_g/255.,
            param1.gt_color_b/255.,
            "gt");
        object1.publish_marker(
            tracker1.belief().mean(),
            2,
            param1.belief_color_r/255.,
            param1.belief_color_g/255.,
            param1.belief_color_b/255.,
            "belief");

        object2.publish_marker(
            object2.pose,
            1,
            param2.gt_color_r/300.,
            param2.gt_color_g/300.,
            param2.gt_color_b/300.,
            "gt");
        object2.publish_marker(
            tracker2.belief().mean(),
            2,
            param2.belief_color_r/300.,
            param2.belief_color_g/300.,
            param2.belief_color_b/300.,
            "belief");
        ros::Duration(0.001).sleep();
        ros::spinOnce();
    }

    PV(tracker1.filter().name());
    PV(tracker2.filter().name());

    if (param1.step_through) std::cin.get();


    while(ros::ok())
    {
        /* ------------------------------ */
        /* - Animate & Render           - */
        /* ------------------------------ */
        object1.animate();
        object2.animate();

        /* ------------------------------ */
        /* - Simulate observation       - */
        /* ------------------------------ */
        object1.render(ground_truth);

        y1 = ground_truth;

        /* ------------------------------ */
        /* - Filter                     - */
        /* ------------------------------ */
//        tracker1.filter_once(y1);
//        tracker2.filter_once(y2);

        bool do_print = false;

        {
            auto& belief1 = tracker1.belief_;
            auto& belief2 = tracker2.belief_;

            State1 old_pose1 = belief1.mean();
            State2 old_pose2 = belief2.mean();

            if (do_print)  PVT(old_pose1);
            if (do_print) PVT(old_pose2);
            if (param1.step_through) std::cin.get();

            Tracker1::Builder::set_nominal_pose(tracker1.filter(), old_pose1);
            Tracker2::Builder::set_nominal_pose(tracker2.filter(), old_pose2);

            State1 zero_pose1 = belief1.mean();
            State2 zero_pose2 = belief2.mean();

            zero_pose1.set_zero_pose();
            zero_pose2.set_zero_pose();

            if (do_print) PVT(zero_pose1);
            if (do_print) PVT(zero_pose2);
            if (param1.step_through) std::cin.get();

            belief1.mean(zero_pose1);
            belief2.mean(zero_pose2);

            if (do_print) PVT(belief1.mean());
            if (do_print) PVT(belief2.mean());
            if (param1.step_through) std::cin.get();

            tracker1.filter_.predict(belief1, Tracker1::Input::Zero(), belief1);
            tracker2.filter_.predict(belief2, Tracker2::Input::Zero(), belief2);

            std::cout << "Prediction result " << std::endl;
            if (do_print) PVT(belief1.mean());
            if (do_print) PVT(belief2.mean());
            if (do_print) PVT(belief1.covariance());
            if (do_print) PVT(belief2.covariance());
            if (param1.step_through) std::cin.get();

            tracker1.filter_.update(belief1, y1, belief1);
            tracker2.filter_.update(belief2, y1, belief2);

            std::cout << "Update result " << std::endl;
            if (do_print) PVT(belief1.mean());
            if (do_print) PVT(belief2.mean());
            if (do_print) PVT(belief1.covariance());
            if (do_print) PVT(belief2.covariance());
            if (param1.step_through) std::cin.get();

            State1 new_pose1 = belief1.mean();
            State2 new_pose2 = belief2.mean();

            new_pose1.set_zero_pose();
            new_pose2.set_zero_pose();

            if (do_print) PVT(new_pose1);
            if (do_print) PVT(new_pose2);
            if (param1.step_through) std::cin.get();

            new_pose1.orientation() = belief1.mean().orientation() * old_pose1.orientation();
            new_pose2.orientation() = belief2.mean().orientation() * old_pose2.orientation();

            std::cout << "orientation update result " << std::endl;
            if (do_print) PVT(new_pose1);
            if (do_print) PVT(new_pose2);
            if (param1.step_through) std::cin.get();

            new_pose1.position() = belief1.mean().position() + old_pose1.position();
            new_pose2.position() = belief2.mean().position() + old_pose2.position();

            std::cout << "position update result " << std::endl;
            if (do_print) PVT(new_pose1);
            if (do_print) PVT(new_pose2);
            if (param1.step_through) std::cin.get();

            belief1.mean(new_pose1);
            belief2.mean(new_pose2);

            std::cout << "final belief " << std::endl;
            if (do_print) PVT(belief1.mean());
            if (do_print) PVT(belief2.mean());
            if (param1.step_through) std::cin.get();
        }


        PVT(tracker1.belief().mean());
        PVT(tracker2.belief().mean());

        PV(tracker1.belief().covariance());
        PV(tracker2.belief().covariance());

        /* ------------------------------ */
        /* - Visualize objects          - */
        /* ------------------------------ */
        object1.publish_marker(
            object1.pose,
            1,
            param1.gt_color_r/255.,
            param1.gt_color_g/255.,
            param1.gt_color_b/255.,
            "gt");
        object1.publish_marker(
            tracker1.belief().mean(),
            2,
            param1.belief_color_r/255.,
            param1.belief_color_g/255.,
            param1.belief_color_b/255.,
            "belief");

        object2.publish_marker(
            object2.pose,
            1,
            param2.gt_color_g2/255.,
            param2.gt_color_r2/255.,
            param2.gt_color_b2/255.,
            "gt");
        object2.publish_marker(
            tracker2.belief().mean(),
            2,
            param2.belief_color_r2/255.,
            param2.belief_color_g2/255.,
            param2.belief_color_b2/255.,
            "belief");

        ros::spinOnce();

        /* ------------------------------ */
        /* - Pause                      - */
        /* ------------------------------ */
        if (param1.step_through) std::cin.get();
    }

    return 0;
}
