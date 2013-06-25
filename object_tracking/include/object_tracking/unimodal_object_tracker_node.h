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
#include <uscauv_common/simple_math.h>
#include <uscauv_common/tic_toc.h>
#include <auv_msgs/MatchedShape.h>
#include <auv_msgs/MatchedShapeArray.h>

#include <cmath>

/// linalg
#include <Eigen/LU>

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

/// template arguments are dims for state/update/control vectors
typedef uscauv::LinearKalmanFilter<8>   _ObjectKalmanFilter;
typedef std::vector<_ObjectKalmanFilter> _KalmanFilterVector;
typedef _ObjectKalmanFilter::Control<8> _FullStateControl;
typedef _ObjectKalmanFilter::Update<4>  _PositionUpdate;
typedef _ObjectKalmanFilter::Update<2>  _FlowVelocityUpdate;

/// TODO: Sort filters_ based on uncertainty
struct ObjectTrackerStorage
{
  std::vector<_ObjectKalmanFilter> filters_;
  _FullStateControl::CovarianceType control_cov_;
  _PositionUpdate::CovarianceType   update_cov_;
  _ObjectKalmanFilter::StateMatrix  initial_cov_;
  double ideal_radius_;
  bool tracked_;
  std::string name_;
  ros::Time last_predict_time_;
  double uncertainty_radius_;
  double filter_kill_thresh_;
};

typedef std::map<std::string, std::string>          _NamedAttributeMap;
typedef std::map<std::string, ObjectTrackerStorage> _AttributeTrackerMap;

/** 
 * Gaussian pdf, but we take the modulus of term 4 because it's a rotatation.
 * We include the determinant because we want to compare probabilities for
 * different filters with different covariances. The 2pi term is unneccessary
 */
static double getGaussianPDFPosition(_PositionUpdate::VectorType x,
				     _PositionUpdate::VectorType mean, 
				     _PositionUpdate::CovarianceType cov )
{
  _PositionUpdate::VectorType diff_term = x - mean;
  diff_term(3) = uscauv::ring_distance<double>( diff_term(3), 0, uscauv::TWO_PI );

  double const md = diff_term.transpose() * cov.inverse() * diff_term;
  double const det = Eigen::PartialPivLU<_PositionUpdate::CovarianceType>(cov).determinant();
  
  return exp(-0.5*md) / sqrt( pow(uscauv::TWO_PI, 4)*det );
}

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
  _PositionUpdate::TransitionType measurement_transition_;
  /* _ObjectKalmanFilter::StateMatrix state_transition_; */

  /// other
  _CameraInfo last_camera_info_;
  image_geometry::PinholeCameraModel camera_model_;
  
 public:
 UnimodalObjectTrackerNode(): BaseNode("UnimodalObjectTracker"), 
    MultiReconfigure( ros::NodeHandle("model/objects") ), /// resolves below node namespaces
    nh_rel_("~"), object_ns_("model/objects")
    {
    }

  /** 
   * For each matched shape corresponding to a tracked object, reproject to 3d and use
   * as a measurement update for the object's kalman filter
   * 
   * @param msg WHat it is
   */
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

	/// Get measurement update params, then update
	_PositionUpdate::VectorType update_mean;
	update_mean << 
	  camera_to_object.x(),
	  camera_to_object.y(), 
	  camera_to_object.z(),
	  shape_it->theta;

	int idx = 0;
	int max_idx = -1;
	double max_prob = 0;
	_KalmanFilterVector & filters = storage.filters_;
	int neighbors = 0;
	for(_KalmanFilterVector::iterator filter_it = filters.begin(); filter_it != filters.end();
	    ++filter_it, ++idx )
	  {
	    double const d = getGaussianPDFPosition( update_mean, measurement_transition_*filter_it->state_, measurement_transition_ * filter_it->cov_ * measurement_transition_.transpose() );
	    ROS_INFO("PDF val: %f", d);

	    if( d >= storage.uncertainty_radius_ && d > max_prob )
	      {
		max_prob = d;
		max_idx = idx;
		neighbors++;
	      }	    
	  }
	ROS_INFO("Found %d neighbor filters.", neighbors);
	/// Spawn a new filter if none of the current filters are a good match for the measurement
	if( max_idx == -1 )
	  {
	    _ObjectKalmanFilter::StateVector initial_state = measurement_transition_.transpose() * update_mean;

	    storage.filters_.push_back( _ObjectKalmanFilter( initial_state, storage.initial_cov_ ) );
	    ROS_INFO_STREAM("Spawned filter ( " << initial_state.transpose() << " ).");
	  }
	else
	  {
	    storage.filters_.at(max_idx).update<4>( update_mean, storage.update_cov_, 
						    measurement_transition_ );
	  }
	

	/// TODO: Compute modified mahalanobis distance between mean and measurement
	/* _PositionUpdate::VectorType diff_term =  */
	/*   update_mean - measurement_transition_ * storage.filter_.state_; */
	/* /// Rotation lives in modular space */
	/* diff_term(3) = uscauv::ring_distance<double>( diff_term(3), 0, uscauv::TWO_PI ); */

	/* _PositionUpdate::CovarianceType state_pos_cov =  */
	/*   measurement_transition_ * storage.filter_.cov_ * measurement_transition_.transpose(); */

	/* double mahalanobis_pos = diff_term.transpose() * state_pos_cov.inverse() * diff_term; */
	
	/* if( mahalanobis_pos > storage.mahalanobis_thresh_pos_ ) */
	/*   { */
	/*     ROS_INFO("Got mahalanobis distance %f. Discarding...", mahalanobis_pos); */
	/*     continue; */
	/*   } */
	
	/* storage.filter_.update<4>( update_mean, storage.update_cov_, */
	/* 			   measurement_transition_ ); */
	
	/* /// update time */
	/* storage.last_update_time_ = msg->header.stamp; */
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
    _FullStateControl::CovarianceType control_cov;
    _PositionUpdate::CovarianceType   update_cov;
    _ObjectKalmanFilter::StateMatrix  initial_cov;

    control_cov = _FullStateControl::CovarianceType::Identity() * cvar;
    update_cov  = _PositionUpdate::CovarianceType::Identity() * uvar;
    initial_cov = _ObjectKalmanFilter::StateMatrix::Identity() * ivar;

    ObjectTrackerStorage & tracker = trackers_.at( name_size_color_map_.at( name ) );

    tracker.control_cov_ = control_cov;
    tracker.initial_cov_ = initial_cov;
    tracker.update_cov_ = update_cov;

    tracker.uncertainty_radius_ = config.uncertainty_radius;
    tracker.filter_kill_thresh_ = config.filter_kill_thresh;

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

    measurement_transition_ << 
      _PositionUpdate::CovarianceType::Identity(),
      _PositionUpdate::CovarianceType::Zero();

    /* state_transition_ <<  */
    /*   _PositionUpdate::CovarianceType::Identity(), */
    /*   _PositionUpdate::CovarianceType::Identity(), */
    /*   _PositionUpdate::CovarianceType::Zero(), */
    /*   _PositionUpdate::CovarianceType::Identity(); */

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
	tracker.last_predict_time_ = ros::Time::now();

	/// have to add the new object before or the lookup in the reconfigure callback will fail to find it
	trackers_[ attr ] = tracker;
	   
	/// set up reconfigure (loads tracker with control_cov and initial_cov params)
	addReconfigureServer<_TrackedObjectConfig>
	  ( object_it->first, std::bind( &UnimodalObjectTrackerNode::updateTrackerParams, this,
					 std::placeholders::_1, object_it->first ) );

	/// initialize kalman filter at location (0,0,0,0) with diagonal covariance set by reconfigure
	/// All transforms are set to identity
	/* ObjectTrackerStorage & tracker_added = trackers_.at(attr); */
	/* tracker_added.filter_ = _ObjectKalmanFilter( _ObjectKalmanFilter::StateVector::Zero(), */
	/* 					     tracker_added.initial_cov_ ); */

	/* tracker_added.last_predict_time_ = ros::Time::now();	 */

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
	/* _ObjectKalmanFilter::StateVector const & state = storage.filter_.state_; */
	  
	if( !storage.tracked_ )
	  continue;

	/// Control input step
	ros::Time now = ros::Time::now();
	double dt = (now - storage.last_predict_time_).toSec();
	storage.last_predict_time_ = now;
	_ObjectKalmanFilter::StateMatrix state_transition;
	/// The second term is respondible for integrating velocity and adding to pos.
	state_transition <<
	  _PositionUpdate::CovarianceType::Identity(),
	  _PositionUpdate::CovarianceType::Identity() * dt, 
	  _PositionUpdate::CovarianceType::Zero(),
	  _PositionUpdate::CovarianceType::Identity();
	
	// ################################################################
	// Remove filters whose variance exceeds a threshold ##############
	// ################################################################

	int idx = 0;
	int min_idx = 0;
	double min_det = -1;
	_KalmanFilterVector & filters = storage.filters_;
	_KalmanFilterVector surviving_filters;
	for(_KalmanFilterVector::iterator filter_it = filters.begin(); filter_it != filters.end();
	    ++filter_it, ++idx )
	  {
	    /// no control input
	    filter_it->predict<8>( _FullStateControl::VectorType::Zero(),
				   storage.control_cov_, state_transition );
	    
	    double const det = Eigen::PartialPivLU<_ObjectKalmanFilter::StateMatrix>( filter_it->cov_ ).determinant();
	    if( det <= storage.filter_kill_thresh_ )
	      {
		surviving_filters.push_back( *filter_it );
		
		if( det < min_det || min_det < 0 )
		  {
		    min_det = det;
		    min_idx = idx;
		  }
		++idx;
	      }
	    else
	      {
		ROS_INFO_STREAM("Killed filter ( " << filter_it->state_.transpose() << " ) Det: " << det << ".");
	      }
	  }
	storage.filters_ = surviving_filters;
	    
	// ################################################################
	// Publish filter estimates. Lowest variance filter gets primary tf
	// ################################################################
	
	idx = 0;
	int aux_idx = 0;    
	for(_KalmanFilterVector::iterator filter_it = filters.begin(); filter_it != filters.end();
	    ++filter_it, ++idx)
	  {
    
	    _ObjectKalmanFilter::StateVector const & state = filter_it->state_;
	    
	    tf::Vector3 camera_to_object_vec = tf::Vector3( state(0), state(1), state(2) );
	    /// setRPY uses R=around X, P=around Y, Y=around Z, so we are rotating around Z
	    tf::Quaternion camera_to_object_quat;
	    camera_to_object_quat.setRPY( 0, 0, state(3));
	    
	    tf::Transform camera_to_object_tf = tf::Transform( camera_to_object_quat,
							       camera_to_object_vec );

	    std::string frame_name;
	    
	    if( idx == min_idx )
	      frame_name = std::string( "object/" + storage.name_ );
	    else
	      {
		std::stringstream ss;
		ss << "object/" << storage.name_ << "aux" << aux_idx;
		frame_name = ss.str();
		++aux_idx;
	      }
	    /// transform from camera to "object/<object name>"
	    /// TODO: Flesh out tracking timeout logic. 
	    /// Currently, transforms only timeout in rviz due to last_update_time_ being too old
	    /// TODO: per above change time to now(), don't let old objects get to this line
	    tf::StampedTransform output( camera_to_object_tf, storage.last_predict_time_,
					 last_camera_info_.header.frame_id, frame_name );
	      
	    object_transforms.push_back( output ); 
	  }

      }

    object_broadcaster_.sendTransform( object_transforms );
    return;
  }
    
};
    
#endif // USCAUV_OBJECTTRACKING_UNIMODALOBJECTTRACKER
