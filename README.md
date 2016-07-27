# ROS Depth Based Object Tracking Library (dbot_ros)

This package extends the dbot library by ros node applications which run the trackers within the ros eco-system.


# Requirements
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

