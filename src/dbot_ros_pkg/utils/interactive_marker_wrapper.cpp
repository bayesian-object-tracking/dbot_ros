#include <dbot_ros_pkg/utils/interactive_marker_wrapper.hpp>
#include <dbot_ros_pkg/utils/ros_interface.hpp>
#include <dbot/states/free_floating_rigid_bodies_state.hpp>

namespace im {

  InteractiveMarkerWrapper::InteractiveMarkerWrapper()
    : nh_("~")
    , server_("object_marker")
    , button_clicked_(false)
  {
    ri::ReadParameter("object_names", object_names_, nh_);
    n_objects_ = object_names_.size();
    marker_positions_.resize(n_objects_);
    marker_orientations_.resize(n_objects_);
        
    for(size_t i = 0; i < object_names_.size(); i++)
      {
	// create an interactive marker for our server
	visualization_msgs::InteractiveMarker int_marker;
	std::string name = std::to_string(i);
	// TODO add frame id either to config or as a parameters of the constructor
	createInteractiveMarker("/XTION", name, "Click on object when satisfied with initialisation", int_marker);
        
	addObjectControl(object_names_[i], int_marker);
	add6DoFControl(int_marker);
      
	// add the interactive marker to our collection &
	// tell the server to call processFeedback() when feedback arrives for it
	server_.insert(int_marker, boost::bind(&InteractiveMarkerWrapper::processFeedback, this, _1));
      }
    
    // 'commit' changes and send to all clients
    server_.applyChanges();
  }

  bool InteractiveMarkerWrapper::initializeObjects()
  {
    if(button_clicked_)
      {
	button_clicked_ = false;
	return true;
      }
    
    return false;
  }
  
  std::vector<Eigen::VectorXd> InteractiveMarkerWrapper::getMarkerPose()
  {
    boost::mutex::scoped_lock lock(mutex_);
    
    ff::FreeFloatingRigidBodiesState<-1> state(n_objects_);
    
    for( int i=0; i<n_objects_; ++i)
      {
	state.quaternion(marker_orientations_[i],i);
	state.position(i) = marker_positions_[i];
      }
    
    //std::vector<Eigen::VectorXd> initial_states(1,state);
    //return initial_states;
    return std::vector<Eigen::VectorXd>(1,state);
  }

  void InteractiveMarkerWrapper::createInteractiveMarker(const std::string frame_id,
							 const std::string name,
							 const std::string description,
							 visualization_msgs::InteractiveMarker &int_marker)
  {
    int_marker.header.frame_id = frame_id;
    int_marker.header.stamp=ros::Time::now();
    int_marker.name = name;
    int_marker.description = description;
    int_marker.scale = 0.3;
    // position interactive marker by default 1 m in front
    // of camera in viewing condition
    int_marker.pose.position.x = 0.0;
    int_marker.pose.position.y = 0.0;
    int_marker.pose.position.z = 1.0;
  }
  
  void InteractiveMarkerWrapper::addObjectControl(const std::string object_name,
						  visualization_msgs::InteractiveMarker &int_marker)
  {
    std::string object_model_path =
      "package://dbot_ros_pkg/object_models/" + object_name + ".obj";
    // create a grey box marker
    visualization_msgs::Marker object_marker;
    object_marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    object_marker.mesh_resource = object_model_path;
    object_marker.scale.x = 1.0;
    object_marker.scale.y = 1.0;
    object_marker.scale.z = 1.0;
    object_marker.color.r = 0.5;
    object_marker.color.g = 0.5;
    object_marker.color.b = 0.5;
    object_marker.color.a = 1.0;
      
    // create a non-interactive control which contains the object
    visualization_msgs::InteractiveMarkerControl object_control;
    object_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
    object_control.name = "button_control";
    object_control.always_visible = true;
    object_control.markers.push_back( object_marker );
    
    // add the control to the interactive marker
    int_marker.controls.push_back( object_control );
  }
  
  void InteractiveMarkerWrapper::add6DoFControl(visualization_msgs::InteractiveMarker &int_marker)
  {
    visualization_msgs::InteractiveMarkerControl control;
    control.orientation.w = 1;
    control.orientation.x = 1;
    control.orientation.y = 0;
    control.orientation.z = 0;
    control.name = "rotate_x";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_x";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 1;
    control.orientation.z = 0;
    control.name = "rotate_z";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_z";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 0;
    control.orientation.z = 1;
    control.name = "rotate_y";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_y";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);
  }
  
  void InteractiveMarkerWrapper::processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
  {
    boost::mutex::scoped_lock lock(mutex_);

    // get marker_id
    int i = std::stoi(feedback->marker_name);

    // get position and orientation
    Eigen::Vector3d pos(feedback->pose.position.x, feedback->pose.position.y, feedback->pose.position.z);
    marker_positions_[i] = pos;
    
    Eigen::Quaternion<double> quat(feedback->pose.orientation.w,
				   feedback->pose.orientation.x,
				   feedback->pose.orientation.y,
				   feedback->pose.orientation.z);
    marker_orientations_[i] = quat;

    if(feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK)
      button_clicked_ = true;
    
  }

}
