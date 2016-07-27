# ROS Depth Based Object Tracking Library (dbot_ros)

This package extends the [dbot](https://github.com/bayesian-object-tracking/dbot) library by ros node applications which run the trackers within the ros eco-system. The main content of this package are two tracker nodes for the trackers provided in dbot. Additionally, the package contains a tracker service which is based on the particle filter based tracker.
All trackers require object mesh models in Wavefront (.obj) format.

# Requirements
 * Kinect or XTION depth sensor
 * Ubuntu 12.04
 * C++0x or C++11 Compiler (gcc-4.6 or later)
 * [CUDA](https://developer.nvidia.com/cuda-downloads) 6.5 or later (optional)
 
## Dependecies
 * [dbot](https://github.com/bayesian-object-tracking/dbot)
 * [dbot_ros_msgs](https://github.com/bayesian-object-tracking/dbot_ros_msgs)
 * [opi](https://github.com/bayesian-object-tracking/opi)
 * [osr](https://github.com/bayesian-object-tracking/osr)
 * [Filtering library](https://github.com/filtering-library/fl) (fl)
 * [Eigen](http://eigen.tuxfamily.org/) 3.2.1 or later
 
 
# Compiling

     $ cd $HOME
     $ mkdir -p projects/tracking/src  
     $ cd projects/tracking/src
     $ git clone git@github.com:filtering-library/fl.git
     $ git clone git@github.com:bayesian-object-tracking/dbot.git
     $ git clone git@github.com:bayesian-object-tracking/dbot_ros_msgs.git
     $ git clone git@github.com:bayesian-object-tracking/opi.git
     $ git clone git@github.com:bayesian-object-tracking/osr.git
     $ git clone git@github.com:bayesian-object-tracking/dbot_ros.git
     $ cd ..
     $ catkin_make -DCMAKE_BUILD_TYPE=Release -DDBOT_BUILD_GPU=On

If no CUDA enabled device is available, you can deactivate the GPU implementation via 

     $ catkin_make -DCMAKE_BUILD_TYPE=Release -DDBOT_BUILD_GPU=Off

# Configuration
The configuration files are located in

     $ cd $HOME/projects/tracking
     $ source devel/setup.bash
     $ roscd dbot_ros
     $ cd config
     $ ls
     $ ... camera.yaml  
     $ ... object_tracker_services.yaml  
     $ ... object.yaml  
     $ ... rbc_particle_filter_tracker.yaml  
     $ ... rms_gaussian_filter_object_tracker.yaml

# Camera configuration (camera.yaml)
The camera configuration file camera.yaml contains the ros depth image topic and camera info topic names

     depth_image_topic: /XTION/depth/image
     camera_info_topic: /XTION/depth/camera_info 

Adjust the topic names if needed.

# Object configuration (object.yaml)
The trackers assume that the tracked object models exist somewhere as a catkin package in your workspace `$HOME/projects/tracking`. The object.yaml file specifies where to find the mesh.obj file of the object you want to track

     object:
       package:    my_object_model_package
       directory:  model
       meshes:     [ duck.obj ]



