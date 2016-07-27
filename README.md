# ROS Depth Based Object Tracking Library (dbot_ros)

This package extends the [dbot](https://github.com/bayesian-object-tracking/dbot) library by ros node applications which run the trackers within the ros eco-system. The main content of this package are two tracker nodes for the trackers provided in dbot. Additionally, the package contains a tracker service which is based on the particle filter based tracker.
All trackers require object mesh models in Wavefront (.obj) format.

# Requirements
 * Kinect or XTION depth sensor
 * Ubuntu 12.04
 * C++0x or C++11 Compiler (gcc-4.6 or later)
 * [CUDA](https://developer.nvidia.com/cuda-downloads) 6.5 or later (optional)
 
### Dependecies
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

### Camera configuration (camera.yaml)
The camera configuration file camera.yaml contains the ros depth image topic and camera info topic names

     depth_image_topic: /XTION/depth/image
     camera_info_topic: /XTION/depth/camera_info 

Adjust the topic names if needed.

### Object configuration (object.yaml)
The trackers assume that the tracked object models exist somewhere as a catkin package in your workspace `$HOME/projects/tracking`. The object.yaml file specifies where to find the mesh.obj file of the object you want to track

     object:
       package:    my_object_model_package
       directory:  model
       meshes:     [ duck.obj ]

### Particle filter config (rbc_particle_filter_tracker.yaml)

Here you won't need to adjust most of the variables. An important one is whether you want to utilize the GPU or not
     particle_filter:
       use_gpu: true 

If GPU support is not availabe, `set use_gpu: false` to run the tracker on the CPU.

### Gaussian filter config (rms_gaussian_filter_tracker.yaml)
The Gaussian filter is a CPU only tracker. You may adjust the filter sensitivity or accuracy by adjusting the noise parameters of the object state transition and observation models. However, the provided default are resonable values. 

    object_transition:
      linear_sigma: 0.002 
      angular_sigma: 0.01 
      velocity_factor: 0.8

    observation:
      fg_noise_std: 0.001 

The provided values are determined for models with time discretized of 33ms given that the depth camera provides images in 30 frames per second.

# Running the trackers

For all trackers launch the ROS OpenNI camera node to publish the depth camera and run ROS `rviz` visualization tool. Add a point cloud display in rviz. This step is required to initialize the trackers.

### Running and Initializing the Particle Filter Based Tracker

     roslaunch dbot_ros rbc_particle_filter_tracker.launch

Once launched, you will have to add an `Interactive Marker` in rviz to initialize the tracker. For that, align the displayed interactive marker with the object's point cloud and click on the object to start the tracker. Finally, add a `Marker` and select the `/rbc_particle_filter_tracker/object_model` topic to display the tracked object.

### Running and Initializing the Gaussian Filter Based Tracker
 The procedure is the same as for the particle filter tracker described above.
 
     roslaunch dbot_ros rms_gaussian_filter_tracker.launch

The object Marker topic has to be changed to `/rms_gaussian_filter_tracker/object_model` in order to display the tracked object.

### Running the Particle Filter via ROS Service

Again the setup is the same as above except the initialization is different. 

     roslaunch dbot_ros object_tracker_service.launch

The service expects a call with the ros message type `ObjectState.msg` located in `dbot_ros_msgs` package
    # name of the object
    string name   
    # Object resource identifier
    ObjectOri ori
    # Object's initial pose
    geometry_msgs/PoseStamped pose

The `ObjectOri.msg` message content is the same as the object.yaml config file
    string package
    string directory
    string name

Once the service is running, you can use ros service call with the `RunObjectTracker.srv` (located in dbot_ros_msgs) to trigger the service to track the desired object. You can call the service either for your own c++ ros node, a python node
or using the `roservice call` command.


