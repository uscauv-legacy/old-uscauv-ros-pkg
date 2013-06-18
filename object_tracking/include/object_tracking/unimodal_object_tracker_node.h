/***************************************************************************
 *  include/object_tracking/unimodal_object_tracker_node.h
 *  --------------------
 *
 *  Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Dylan Foster
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are
 *  met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following disclaimer
 *    in the documentation and/or other materials provided with the
 *    distribution.
 *  * Neither the name of USC AUV nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 **************************************************************************/


#ifndef USCAUV_OBJECTTRACKING_UNIMODALOBJECTTRACKER
#define USCAUV_OBJECTTRACKING_UNIMODALOBJECTTRACKER

// ROS
#include <ros/ros.h>

#include <tf/LinearMath/Transform.h>
#include <tf/transform_broadcaster.h>

// general uscauv
#include <uscauv_common/base_node.h>
#include <uscauv_common/multi_reconfigure.h>
#include <uscauv_common/param_loader.h>
#include <uscauv_common/image_geometry.h>
#include <auv_msgs/MatchedShape.h>
#include <auv_msgs/MatchedShapeArray.h>

/// object tracking
#include <object_tracking/kalman_filter.h>
#include <object_tracking/TrackedObjectConfig.h>

#include <sensor_msgs/CameraInfo.h>
#include <image_geometry/pinhole_camera_model.h>


typedef auv_msgs::MatchedShape _MatchedShape;
typedef auv_msgs::MatchedShapeArray _MatchedShapeArray;

typedef sensor_msgs::CameraInfo _CameraInfo;

typedef std::map<std::string, XmlRpc::XmlRpcValue> _NamedXmlMap;
typedef XmlRpc::XmlRpcValue _XmlVal;

typedef object_tracking::TrackedObjectConfig _TrackedObjectConfig;

typedef uscauv::LinearKalmanFilter<4, 4, 4> _ObjectKalmanFilter;

struct ObjectTrackerStorage
{
  _ObjectKalmanFilter filter_;
  _ObjectKalmanFilter::ControlMatrix control_cov_;
  _ObjectKalmanFilter::UpdateMatrix update_cov_;
  _ObjectKalmanFilter::StateMatrix initial_cov_;
  double ideal_radius_;
  bool tracked_;
  std::string name_;
  ros::Time last_update_time_;
};

typedef std::map<std::string, std::string>          _NamedAttributeMap;
typedef std::map<std::string, ObjectTrackerStorage> _AttributeTrackerMap;

/// TODO: Add support for start/stop/reset tracking service
class UnimodalObjectTrackerNode: public BaseNode, public MultiReconfigure
{
 private:
  /// ros
  ros::NodeHandle nh_rel_;
  ros::Subscriber matched_shape_sub_, camera_info_sub_;
  tf::TransformBroadcaster object_broadcaster_;
  
  std::string const object_ns_;
  std::string depth_method_;

  /// algorithmic
  _NamedAttributeMap name_size_color_map_;
  _AttributeTrackerMap trackers_;

  /// other
  _CameraInfo last_camera_info_;
  image_geometry::PinholeCameraModel camera_model_;
  
 public:
 UnimodalObjectTrackerNode(): BaseNode("UnimodalObjectTracker"), 
    MultiReconfigure( ros::NodeHandle("model/objects") ), /// resolves below node namespaces
    nh_rel_("~"), object_ns_("model/objects")
    {
    }

  void matchedShapeCallback( _MatchedShapeArray::ConstPtr const & msg )
  {
    if ( msg->header.frame_id != last_camera_info_.header.frame_id )
      {
	ROS_WARN( "Matched shape frame does not match camera frame. Discarding message...");
	return;
      }
    
    for( std::vector<_MatchedShape>::const_iterator shape_it= msg->shapes.begin();
	 shape_it != msg->shapes.end(); ++shape_it)
      {

	std::string const & attr = shape_it->type + "/" + shape_it->color;
	
	/// Quit if we don't have the object or aren't tracking it
	_AttributeTrackerMap::iterator tracker = trackers_.find( attr );
	if( tracker == trackers_.end() || !tracker->second.tracked_ ) continue;

	// ################################################################
	// Project to 3d ##################################################
	// ################################################################
	ObjectTrackerStorage & storage = tracker->second;
	tf::Vector3 camera_to_object;	

	if( !camera_model_.initialized() )
	  {
	    ROS_WARN( "Camera model is not ready.");
	    return;
	  }

	if( depth_method_ == "monocular" )
	  {
	    camera_to_object = 
	      uscauv::reprojectObjectTo3d( camera_model_, cv::Point2d( shape_it->x, shape_it->y),
					   shape_it->scale, storage.ideal_radius_ );
	  }
	
	// ################################################################
	// Update filter ##################################################
	// ################################################################

	/// predict no change with covariance from param
	/* storage.filter_.predict( _ObjectKalmanFilter::ControlVector::Zero(), */
	/* 			 storage.control_cov_ ); */
	
	/// Get measurement update params, then update
	_ObjectKalmanFilter::UpdateVector update_mean;
	update_mean << 
	  camera_to_object.x(),
	  camera_to_object.y(), 
	  camera_to_object.z(),
	  shape_it->theta;
	  
	/// boost::array<double, 16>
	/* _MatchedShape::_covariance_type const & c = shape_it->covariance; */
	
	/* _ObjectKalmanFilter::UpdateMatrix update_cov; */
	/* update_cov << */
	/*   c[0],  c[1],  c[2],  c[3],  */
	/*   c[4],  c[5],  c[6],  c[7],  */
	/*   c[8],  c[9],  c[10], c[11],  */
	/*   c[12], c[13], c[14], c[15]; */

	/* storage.filter_.update( update_mean, update_cov ); */
	storage.filter_.update( update_mean, storage.update_cov_ );
	
	/// update time
	storage.last_update_time_ = msg->header.stamp;
	/* ROS_INFO_STREAM( storage.filter_ ); */
      }
  }

  /// cache camera info
  void cameraInfoCallback( _CameraInfo::ConstPtr const & msg )
  {
    /* This is a heuristic */
    if( msg->distortion_model == "" )
      {
	ROS_WARN("Received uninitialized camera info message. Discarding...");
	return;
      }
    last_camera_info_ = *msg;
    camera_model_.fromCameraInfo( last_camera_info_ );
  }

  void updateTrackerParams(_TrackedObjectConfig const & config, std::string const & name)
  {
    double const & cvar = config.predict_variance;
    double const & ivar = config.initial_variance;
    double const & uvar = config.update_variance;
    _ObjectKalmanFilter::ControlMatrix control_cov;
    _ObjectKalmanFilter::StateMatrix initial_cov;
    _ObjectKalmanFilter::UpdateMatrix update_cov;

    /// fancy looking
    control_cov <<
      cvar, 0, 0, 0,
      0, cvar, 0, 0,
      0, 0, cvar, 0,
      0, 0, 0, cvar;

    initial_cov <<
      ivar, 0, 0, 0,
      0, ivar, 0, 0,
      0, 0, ivar, 0,
      0, 0, 0, ivar;

    update_cov <<
      uvar, 0, 0, 0,
      0, uvar, 0, 0,
      0, 0, uvar, 0,
      0, 0, 0, uvar;

    ObjectTrackerStorage & tracker = trackers_.at( name_size_color_map_.at( name ) );

    tracker.control_cov_ = control_cov;
    tracker.initial_cov_ = initial_cov;
    tracker.update_cov_ = update_cov;
    ROS_INFO("Updated tracker params [ %s ].", name.c_str() );
  }

 private:

  // Running spin() will cause this function to be called before the node begins looping the spinOnce() function.
  /// TODO: Catch XML exception
  void spinFirst()
  {
    ros::NodeHandle nh_base;
    bool immediate_tracking;
    _XmlVal xml_objects;
	      
    matched_shape_sub_ = nh_rel_.subscribe("matched_shapes", 10, 
					   &UnimodalObjectTrackerNode::matchedShapeCallback,
					   this);
    camera_info_sub_ = nh_rel_.subscribe("camera_info", 1,
					 &UnimodalObjectTrackerNode::cameraInfoCallback, 
					 this);

    immediate_tracking = uscauv::loadParam<bool>( nh_rel_, "immediate_tracking", false );
    depth_method_ = uscauv::loadParam<std::string>( nh_rel_, "depth_method", "monocular" );

    /// TODO: Add more depth methods
    if( depth_method_ != "monocular" )
      {
	ROS_WARN( "Got depth method [ %s ], but only monocular method is supported. Switching...", 
		  depth_method_.c_str());
	depth_method_ = "monocular";
      }
       
    // ################################################################
    // Load objects definitions from parameter server #################
    // ################################################################
    xml_objects = uscauv::loadParam<uscauv::XmlRpcValue>( nh_base, object_ns_ );
       
    for( _NamedXmlMap::iterator object_it = xml_objects.begin(); 
	 object_it != xml_objects.end(); ++object_it )
      {
	std::string const attr = std::string(object_it->second["shape"]) + "/" +
	  std::string(object_it->second["color"]);
	name_size_color_map_[ object_it->first] = attr;
	   
	ObjectTrackerStorage tracker;
	tracker.name_ = object_it->first;
	tracker.ideal_radius_ = object_it->second["ideal_radius"];
	tracker.tracked_ = immediate_tracking;

	/// have to add the new object before or the lookup in the reconfigure callback will fail to find it
	trackers_[ attr ] = tracker;
	   
	/// set up reconfigure (loads tracker with control_cov and initial_cov params)
	addReconfigureServer<_TrackedObjectConfig>
	  ( object_it->first, std::bind( &UnimodalObjectTrackerNode::updateTrackerParams, this,
	   				 std::placeholders::_1, object_it->first ) );

	/// initialize kalman filter at location (0,0,0,0) with diagonal covariance set by reconfigure
	/// All transforms are set to identity
	ObjectTrackerStorage & tracker_added = trackers_.at(attr);
	tracker_added.filter_ = _ObjectKalmanFilter( _ObjectKalmanFilter::StateVector::Zero(),
						     tracker_added.initial_cov_,
						     _ObjectKalmanFilter::StateMatrix::Identity(),
						     _ObjectKalmanFilter::StateControlMatrix::Identity(),
						     _ObjectKalmanFilter::UpdateStateMatrix::Identity() );
	
	ROS_INFO("Loaded object [ %s ] with attributes [ %s ].", 
		 object_it->first.c_str(), attr.c_str() );
	/* ROS_INFO_STREAM( tracker_added.filter_ ); */
      }
  }  

  // Running spin() will cause this function to get called at the loop rate until this node is killed.
  void spinOnce()
  {
    // ################################################################
    // Publish tf transforms for all objects that are curently tracked
    // ################################################################
    if( !camera_model_.initialized() )
      return;

    std::vector< tf::StampedTransform > object_transforms;
    
    for( _NamedAttributeMap::const_iterator name_it = name_size_color_map_.begin(); 
	 name_it != name_size_color_map_.end(); ++name_it )
      {
	ObjectTrackerStorage & storage = trackers_.at( name_it->second );
	_ObjectKalmanFilter::StateVector const & state = storage.filter_.state_;

	if( !storage.tracked_ )
	  continue;

	/// Control input step
	storage.filter_.predict( _ObjectKalmanFilter::ControlVector::Zero(),
			 storage.control_cov_ );

	tf::Vector3 camera_to_object_vec = tf::Vector3( state(0), state(1), state(2) );
	/// setRPY uses R=around X, P=around Y, Y=around Z, so we are rotating around Z
	tf::Quaternion camera_to_object_quat;
	camera_to_object_quat.setRPY( 0, 0, state(3));

	tf::Transform camera_to_object_tf = tf::Transform( camera_to_object_quat,
							   camera_to_object_vec );
	
	/// transform from camera to "object/<object name>"
	tf::StampedTransform output( camera_to_object_tf, storage.last_update_time_,
				     last_camera_info_.header.frame_id,
				     std::string( "object/" + storage.name_ ) );
	
	object_transforms.push_back( output );
      }
    
    object_broadcaster_.sendTransform( object_transforms );
  }
  
};

#endif // USCAUV_OBJECTTRACKING_UNIMODALOBJECTTRACKER
