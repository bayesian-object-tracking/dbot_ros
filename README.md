# ROS Depth Based Object Tracking Library (dbot_ros)

This package extends the [dbot](https://github.com/bayesian-object-tracking/dbot) library by ros node applications which run the trackers within the ros eco-system. The main content of this package are two tracker nodes for the trackers provided in dbot. Additionally, the package contains a tracker service which is based on the particle filter based tracker.
All trackers require object mesh models in Wavefront (.obj) format. To get started, we recommend that you follow the instructions at https://github.com/bayesian-object-tracking/getting_started.

# Requirements
 * MS Kinect or Asus XTION depth sensor
 * Ubuntu 14.04
 * Tested with [ROS Indigo](http://wiki.ros.org/indigo)
 * c++11 Compiler (gcc-4.7 or later)
 * [CUDA](https://developer.nvidia.com/cuda-downloads) 6.5 or later (optional)
 
## Dependecies
 * [dbot](https://github.com/bayesian-object-tracking/dbot)
 * [dbot_ros_msgs](https://github.com/bayesian-object-tracking/dbot_ros_msgs)
 * [Filtering library](https://github.com/filtering-library/fl) (fl)
 * [Eigen](http://eigen.tuxfamily.org/) 3.2.1 or later
 
 
# Compiling
```bash
$ cd $HOME
$ mkdir -p projects/tracking/src  
$ cd projects/tracking/src
$ git clone git@github.com:filtering-library/fl.git
$ git clone git@github.com:bayesian-object-tracking/dbot.git
$ git clone git@github.com:bayesian-object-tracking/dbot_ros_msgs.git
$ git clone git@github.com:bayesian-object-tracking/dbot_ros.git
$ cd ..
$ catkin_make -DCMAKE_BUILD_TYPE=Release -DDBOT_BUILD_GPU=On
```
If no CUDA enabled device is available, you can deactivate the GPU implementation via 
```bash
$ catkin_make -DCMAKE_BUILD_TYPE=Release -DDBOT_BUILD_GPU=Off
```
# Configuration
The configuration files are located in
```bash
$ cd $HOME/projects/tracking
$ source devel/setup.bash
$ roscd dbot_ros
$ cd config
$ ls
$ ... camera.yaml  
$ ... object_tracker_services.yaml  
$ ... object.yaml  
$ ... particle_tracker.yaml  
$ ... gaussian_tracker.yaml
```
## Camera configuration (camera.yaml)
The camera configuration file camera.yaml contains the ros depth image topic and camera info topic names
```yaml
depth_image_topic: /camera/depth/image
camera_info_topic: /camera/depth/camera_info 
```
Adjust the topic names if needed.

#### Object configuration (object.yaml)
The trackers assume that the tracked object models exist somewhere as a catkin package in your workspace `$HOME/projects/tracking`. The object.yaml file specifies where to find the mesh.obj file of the object you want to track
```yaml
object:
  package:    my_object_model_package
  directory:  model
  meshes:     [ duck.obj ]
```
## Particle filter config (particle_tracker.yaml)

Here you won't need to adjust most of the variables. An important one is whether you want to utilize the GPU or not
```yaml
particle_filter:
  use_gpu: true 
```
If GPU support is not availabe, `set use_gpu: false` to run the tracker on the CPU.

## Gaussian filter config (gaussian_tracker.yaml)
The Gaussian filter is a CPU only tracker. You may adjust the filter sensitivity or accuracy by adjusting the noise parameters of the object state transition and observation models. However, the provided default are resonable values. 
```yaml
object_transition:
  linear_sigma: 0.002 
  angular_sigma: 0.01 
  velocity_factor: 0.8

observation:
  fg_noise_std: 0.001 
```
The provided values are determined for models with time discretized of 33ms given that the depth camera provides images in 30 frames per second.

# Running the trackers

For all trackers launch the ROS OpenNI camera node to publish the depth camera and run ROS `rviz` visualization tool. Add a point cloud display in rviz. This step is required to initialize the trackers.

## Running and Initializing the Particle Filter Based Tracker
```bash
$ roslaunch dbot_ros particle_tracker.launch
```
Once launched, you will have to add an `Interactive Marker` in rviz to initialize the tracker. For that, align the displayed interactive marker with the object's point cloud and click on the object to start the tracker. Finally, add a `Marker` and select the `/particle_tracker/object_model` topic to display the tracked object. The tracking estimate is published under the topic `/particle_tracker/object_state`.

## Running and Initializing the Gaussian Filter Based Tracker
 The procedure is the same as for the particle filter tracker described above.
 ```bash
$ roslaunch dbot_ros gaussian_tracker.launch
```
The object Marker topic has to be changed to `/gaussian_tracker/object_model` in order to display the tracked object. The tracking estimate is published under the topic `/gaussian_tracker/object_state`.

## Running the Particle Filter via ROS Service

Again the setup is the same as above except the initialization is different. 
```bash
$ roslaunch dbot_ros object_tracker_service.launch
```
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

### Run Tracker service via `rosservice call `
Here is an example on how to trigger the tracker service using the `rosservice call` command
```bash
$ rosservice call /object_tracker_service \
$ [
$   my_mug_box,
$   [object_meshes, object_models, mugbox.obj],
$   [
$     [0, 0, /XTOIN],
$     [
$       [0, 0, 0.7],
$       [0, 0, 0, 0]
$     ]
$   ]
$ ]"
```

Breaking down the above command line:
rosservice expects 
```bash
$ rosservice <command> <service-topic> "service message value"
```
Here `/object_tracker_service` is the tracker service topic name. What follows is the value of `RunObjectTracker.srv` service definition:

     [ObjectState]
     ObjectState = [name, ObjectOri, geometry_msgs/PoseStamped]
     ObjectOri = [package, directory, object_mesh.obj]
     geometry_msgs/PoseStamped = [header, geometry_msgs/Pose]
     header = [seq, stamp, frame_id]
     geometry_msgs/Pose = [position, orientation]
     position = [x, y, z]
     orientation = [qx, qy, qz, qw]
 
The tracking estimate is published under the topic `/object_tracker_service/object_state`.
 
### Tracker Service via rospy 
Here is a simple listing to trigger the object tracker service which is available at `dbot_ros/scripts/track_object_service_call_example.py`.



```python
#!/usr/bin/env python

import sys
import rospy
from std_msgs.msg import Header
from dbot_ros_msgs.srv import RunObjectTracker
from dbot_ros_msgs.msg import ObjectState
from dbot_ros_msgs.msg import ObjectOri
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped

def track_object(object_name):
  print "Waiting for service..."
  rospy.wait_for_service("/object_tracker_service")

  try:
    run_object_tracker = rospy.ServiceProxy('/object_tracker_service', RunObjectTracker)

    # set initial pose
    pose = Pose(position=Point(0, 0, 0.7),
                orientation=Quaternion(0, 0, 0, 0))

    # set Object resource identifier to find the model
    ori = ObjectOri(package = "object_meshes",
                    directory="object_models",
                    name=object_name + ".obj")

    # construct the ObjectState message for the service call
    object_state = ObjectState(
            name=object_name,
            ori=ori,
            pose=PoseStamped(
                   pose=pose,
                   header=Header(seq=0, stamp=rospy.Time(0), frame_id='')))

    print "Calling tracker service to track the %s object" % object_name
    run_object_tracker(object_state)
  except rospy.ServiceException, e:
      print "Calling object tracker service failed: %s" % e

if __name__ == "__main__":
  track_object(sys.argv[1])
```
You can run the script as follows:
```bash
$ rosrun dbot_ros track_object_service_call_example.py MyDuck
```
Again, the tracking estimate is published under the topic `/object_tracker_service/object_state`.
