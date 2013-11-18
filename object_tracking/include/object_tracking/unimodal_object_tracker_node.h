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
#include <tf/transform_listener.h>

// general uscauv
#include <uscauv_common/base_node.h>
#include <uscauv_common/multi_reconfigure.h>
#include <uscauv_common/param_loader.h>
#include <uscauv_common/image_geometry.h>
#include <uscauv_common/simple_math.h>
#include <uscauv_common/defaults.h>
#include <uscauv_common/tic_toc.h>
#include <uscauv_common/macros.h>
#include <auv_msgs/MatchedShape.h>
#include <auv_msgs/MatchedShapeArray.h>
#include <auv_msgs/TrackedObject.h>
#include <auv_msgs/TrackedObjectArray.h>

#include <cmath>
#include <map>
#include <unordered_set>

/// linalg
#include <Eigen/LU>

/// object tracking
#include <object_tracking/kalman_filter.h>
#include <object_tracking/TrackedObjectConfig.h>
#include <object_tracking/ObjectTrackerConfig.h>

#include <sensor_msgs/CameraInfo.h>
#include <image_geometry/pinhole_camera_model.h>


typedef auv_msgs::MatchedShape _MatchedShape;
typedef auv_msgs::MatchedShapeArray _MatchedShapeArray;

typedef auv_msgs::TrackedObject _TrackedObjectMsg;
typedef auv_msgs::TrackedObjectArray _TrackedObjectArrayMsg;

typedef sensor_msgs::CameraInfo _CameraInfo;

typedef std::map<std::string, XmlRpc::XmlRpcValue> _NamedXmlMap;
typedef XmlRpc::XmlRpcValue _XmlVal;

typedef object_tracking::TrackedObjectConfig _TrackedObjectConfig;
typedef object_tracking::ObjectTrackerConfig _ObjectTrackerConfig;

/// template arguments are dims for state/update/control vectors
typedef uscauv::LinearKalmanFilter<8>   _ObjectKalmanFilter;
typedef _ObjectKalmanFilter::Control<8> _FullStateControl;
typedef _ObjectKalmanFilter::Update<4>  _PositionUpdate;

/// For using estimates from optical flow - not implemented yet
typedef _ObjectKalmanFilter::Update<2>  _FlowVelocityUpdate;

typedef std::unordered_set<std::string> _ColorSet;

/// TODO: Sort filters_ based on uncertainty

struct FilterStorage
{
  _ObjectKalmanFilter filter_;
  std::string color_;
};

typedef std::vector<FilterStorage> _KalmanFilterVector;

struct ObjectTrackerStorage
{
  std::vector<FilterStorage> filters_;
  double ideal_radius_;
  
  std::string type_;
  _ColorSet colors_;
  
  ros::Time last_predict_time_;
  _TrackedObjectConfig config_;
};


/* typedef std::map<std::string, ObjectTrackerStorage> _AttributeTrackerMap; */
typedef std::multimap<std::string, std::string>     _ShapeTrackerMap;
typedef std::pair<_ShapeTrackerMap::iterator, _ShapeTrackerMap::iterator> _ShapeTrackerMapRange;

typedef std::map<std::string, ObjectTrackerStorage> _NamedTrackerMap;

/** 
 * Gaussian pdf, but we take the modulus of term 4 because it's a rotatation.
 * We include the determinant because we want to compare probabilities for
 * different filters with different covariances. The 2pi term is unneccessary
 */
static double getGaussianPDFPosition(_PositionUpdate::VectorType x,
				     _PositionUpdate::VectorType mean, 
				     _PositionUpdate::CovarianceType cov,
				     double const & yaw_symmetry )
{
  _PositionUpdate::VectorType diff_term = x - mean;
  diff_term(3) = uscauv::ring_distance<double>( diff_term(3), 0, yaw_symmetry );

  /// mahalanobis distance
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
  ros::Publisher tracked_object_pub_;
  tf::TransformBroadcaster object_broadcaster_;
  tf::TransformListener tf_listener_;
  
  std::string const object_ns_;
  std::string depth_method_;
  std::string motion_frame_;

  _ObjectTrackerConfig config_;

  _FullStateControl::CovarianceType control_cov_;
  _PositionUpdate::CovarianceType   update_cov_;
  _ObjectKalmanFilter::StateMatrix  initial_cov_;

  /// algorithm
  _ShapeTrackerMap shape_tracker_map_;
  _NamedTrackerMap trackers_;
  _PositionUpdate::TransitionType measurement_transition_;

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

	/// Find all of the trackers that are tracking objects with this shape
	_ShapeTrackerMapRange match_range = shape_tracker_map_.equal_range( shape_it->type );

	if( match_range.first == match_range.second ) continue;


	/// Process the measurement for each compatible tracker
	for( _ShapeTrackerMap::iterator tracker_it = match_range.first; tracker_it != match_range.second;
	     ++tracker_it )
	  {
	    
	    // ################################################################
	    // Project to 3d ##################################################
	    // ################################################################

	    _NamedTrackerMap::iterator tracker = trackers_.find( tracker_it->second );

	    // Shouldn't happen - shape_tracker_map should only contain names of trackers in the map trackers_
	    ROS_ASSERT( tracker != trackers_.end() );

	    ObjectTrackerStorage & storage = tracker->second;

	    // If this particular tracker cannot assume the color of the matched shape, we ignore it
	    if( storage.colors_.find( shape_it->color ) == storage.colors_.end() )
	      continue;
	    
	    tf::Vector3 camera_to_object_vec;	

	    if( !camera_model_.initialized() )
	      {
		ROS_WARN( "Camera model is not ready.");
		return;
	      }

	    if( depth_method_ == "monocular" )
	      {
		camera_to_object_vec = 
		  uscauv::reprojectObjectTo3d( camera_model_, cv::Point2d( shape_it->x, shape_it->y),
					       shape_it->scale, storage.ideal_radius_ );
	      }
	    else
	      {
		ROS_ERROR("Bad depth method.");
		return;
	      }
	
	    // ################################################################
	    // Update filter ##################################################
	    // ################################################################

	    /// Get measurement update params, then update
	    _PositionUpdate::VectorType update_mean;
	    update_mean << 
	      camera_to_object_vec.x(),
	      camera_to_object_vec.y(), 
	      camera_to_object_vec.z(),
	      shape_it->theta;

	    int idx = 0;
	    int max_idx = -1;
	    double max_prob = 0;
	    _KalmanFilterVector & filters = storage.filters_;
	    int neighbors = 0;
	    for(_KalmanFilterVector::iterator filter_it = filters.begin(); filter_it != filters.end();
		++filter_it, ++idx )
	      {
		_ObjectKalmanFilter & filter = filter_it->filter_;
		
		_PositionUpdate::VectorType state_pos = measurement_transition_*filter.state_;
		_PositionUpdate::VectorType diff_term = state_pos - update_mean;

		double const d = getGaussianPDFPosition( update_mean, state_pos, measurement_transition_ * filter.cov_ * measurement_transition_.transpose(), storage.config_.symmetry );
		double const dist_euclidian = diff_term.block(0,0,3,1).norm();
		double const dist_angular = uscauv::ring_distance<double>( diff_term(3), 0, storage.config_.symmetry );
		ROS_DEBUG("PDF val: %0.20f, dist: %f, angle %f", d, dist_euclidian, dist_angular);

		if( dist_euclidian <= storage.config_.exclude_distance
		    && dist_angular <= storage.config_.exclude_angle
		    && d > max_prob )
		  {
		    max_prob = d;
		    max_idx = idx;
		    neighbors++;
		  }	    
	      }
	    ROS_DEBUG("Found %d neighbor filters.", neighbors);
	    /// Spawn a new filter if none of the current filters are a good match for the measurement
	    if( max_idx == -1 )
	      {
		_ObjectKalmanFilter::StateVector initial_state = measurement_transition_.transpose() * update_mean;

		FilterStorage new_filter;
		new_filter.filter_ = _ObjectKalmanFilter( initial_state, initial_cov_ );
		new_filter.color_ = shape_it->color;

		storage.filters_.push_back( new_filter );
		ROS_DEBUG_STREAM("Spawned filter ( " << initial_state.transpose() << " ).");
	      }
	    else
	      {
		FilterStorage & updated_filter = storage.filters_.at(max_idx);
		updated_filter.filter_.update<4>( update_mean, update_cov_, measurement_transition_ );
		updated_filter.color_ = shape_it->color;
	      }
	    
	  } // matched trackers
      } // matched shapes
  } //callback
  
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

  void updateTrackerParams(_TrackedObjectConfig const & config, std::string const & type)
  {
    ObjectTrackerStorage & tracker = trackers_.at( type );

    tracker.config_ = config;

    ROS_INFO("Updated tracker params [ %s ].", type.c_str() );
  }

  void reconfigureCallback( _ObjectTrackerConfig const & config )
  {
    double const & cvar = config.predict_variance;
    double const & ivar = config.initial_variance;
    double const & uvar = config.update_variance;

    control_cov_ = _FullStateControl::CovarianceType::Identity() * cvar;
    update_cov_  = _PositionUpdate::CovarianceType::Identity() * uvar;
    initial_cov_ = _ObjectKalmanFilter::StateMatrix::Identity() * ivar;
    
    config_ = config;
  }

 private:

  // Running spin() will cause this function to be called before the node begins looping the spinOnce() function.
  /// TODO: Catch XML exception
  void spinFirst()
  {
    ros::NodeHandle nh_base;
    _XmlVal xml_objects;

    measurement_transition_ << 
      _PositionUpdate::CovarianceType::Identity(),
      _PositionUpdate::CovarianceType::Zero();


    matched_shape_sub_ = nh_rel_.subscribe("matched_shapes", 10, 
					   &UnimodalObjectTrackerNode::matchedShapeCallback,
					   this);
    camera_info_sub_ = nh_rel_.subscribe("camera_info", 1,
					 &UnimodalObjectTrackerNode::cameraInfoCallback, 
					 this);

    tracked_object_pub_ = nh_base.advertise<_TrackedObjectArrayMsg>("robot/sensors/tracked_objects", 10);

    depth_method_ = uscauv::param::load<std::string>( nh_rel_, "depth_method", "monocular" );
    motion_frame_ = uscauv::param::load<std::string>( nh_rel_, "motion_frame", uscauv::defaults::CM_LINK );
    
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
    xml_objects = uscauv::param::load<XmlRpc::XmlRpcValue>( nh_base, object_ns_ );
       
    for( _NamedXmlMap::iterator object_it = xml_objects.begin(); 
	 object_it != xml_objects.end(); ++object_it )
      {
	/// hack
	if( object_it->first == "global" )
	  continue;
	
	std::string const attr = std::string(object_it->second["shape"]);
	
	_NamedXmlMap xml_colors = uscauv::param::lookup<_NamedXmlMap>(object_it->second, "colors");
	
	ObjectTrackerStorage tracker;
	tracker.type_ = object_it->first;
	tracker.ideal_radius_ = object_it->second["ideal_radius"];
	tracker.last_predict_time_ = ros::Time::now();

	for(_NamedXmlMap::iterator color_it = xml_colors.begin(); color_it != xml_colors.end(); ++color_it)
	  {
	    tracker.colors_.insert( color_it->first );
	  }

	/// have to add the new object before or the lookup in the reconfigure callback will fail to find it
	shape_tracker_map_.insert( std::make_pair( attr, tracker.type_ ));
	trackers_.insert( std::make_pair( tracker.type_, tracker ) );
	
	/* trackers_[ attr ] = tracker; */
	   
	/// set up reconfigure (loads tracker with control_cov and initial_cov params)
	addReconfigureServer<_TrackedObjectConfig>
	  ( object_it->first, std::bind( &UnimodalObjectTrackerNode::updateTrackerParams, this,
					 std::placeholders::_1, tracker.type_ ));


	ROS_INFO("Loaded object [ %s ] with attributes [ %s ].", 
		 object_it->first.c_str(), attr.c_str() );
      }
   
    /// global object tracker settings ( model/objects/global )
    addReconfigureServer<_ObjectTrackerConfig>("global", &UnimodalObjectTrackerNode::reconfigureCallback, this );

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
    _TrackedObjectArrayMsg tracked_objects;
    
    for( _NamedTrackerMap::iterator tracker_it = trackers_.begin(); tracker_it != trackers_.end();
	 ++tracker_it)
      {
	ObjectTrackerStorage & storage = tracker_it->second;

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
	    ++filter_it )
	  {
	    _ObjectKalmanFilter & filter = filter_it->filter_;
	    
	    /// no control input
	    filter.predict<8>( _FullStateControl::VectorType::Zero(),
				   control_cov_, state_transition );
	    
	    double const det = Eigen::PartialPivLU<_ObjectKalmanFilter::StateMatrix>( filter.cov_ ).determinant();
	    if( det <= config_.kill_var )
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
		ROS_DEBUG_STREAM("Killed filter ( " << filter.state_.transpose() << " ) Det: " << det << ".");
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
	    _ObjectKalmanFilter & filter = filter_it->filter_;
	    
	    _ObjectKalmanFilter::StateVector const & state = filter.state_;
	    
	    tf::Vector3 observer_to_object_vec = tf::Vector3( state(0), state(1), state(2) );
	    /// setRPY uses R=around X, P=around Y, Y=around Z, so we are rotating around Z
	    tf::Quaternion observer_to_object_quat;
	    observer_to_object_quat.setRPY( 0, 0, state(3));
	    
	    tf::Transform observer_to_object_tf = tf::Transform( observer_to_object_quat,
								 observer_to_object_vec );
	    
	    tf::StampedTransform motion_to_observer_tf;

	    /// get the transform from the motion frame (CM on the physical robot) to the camera frame
	    if( tf_listener_.canTransform( motion_frame_, last_camera_info_.header.frame_id, ros::Time(0) ))
	      {
		try
		  {
		    tf_listener_.lookupTransform( motion_frame_, last_camera_info_.header.frame_id, ros::Time(0), motion_to_observer_tf );
		  }
		catch(tf::TransformException & ex)
		  {
		    ROS_ERROR( "Caught exception [ %s ] looking up transform", ex.what() );
		    return;
		  }
	      }
	    else 
	      return;

	    tf::Transform motion_to_object_tf = motion_to_observer_tf * observer_to_object_tf;

	    std::string frame_name;
	    
	    if( idx == min_idx )
	      frame_name = std::string( "object/" + storage.type_ );
	    else
	      {
		std::stringstream ss;
		ss << "object/" << storage.type_ << "_hyp" << aux_idx;
		frame_name = ss.str();
		++aux_idx;
	      }

	    /// Add TrackedObject msg for object
	    /// TODO: Don't recalulate det, add color, add children, add covariance for pose
	    double const det = Eigen::PartialPivLU<_ObjectKalmanFilter::StateMatrix>( filter.cov_ ).determinant();
	    if( det <= config_.pass_var )
	      {
		_TrackedObjectMsg object;
		object.variance = det;
		object.symmetry = storage.config_.symmetry;
		object.color = filter_it->color_;
		object.type = storage.type_;

		object.header.frame_id = motion_frame_;
		/// Time of latest filter prediction, not measurement
		object.header.stamp = ros::Time::now();

		if( idx == min_idx )
		  object.is_best_estimate = true;
		else
		  object.is_best_estimate = false;

		tf::poseTFToMsg( motion_to_object_tf, object.pose.pose );
		tracked_objects.objects.push_back( object );
		
		/// transform from camera to "object/<object name>"
		/// TODO: Flesh out tracking timeout logic. 
		/// Currently, transforms only timeout in rviz due to last_update_time_ being too old
		/// TODO: per above change time to now(), don't let old objects get to this line
		tf::StampedTransform output( motion_to_object_tf, storage.last_predict_time_,
					     motion_frame_, frame_name );
		
		object_transforms.push_back( output ); 
		
	      }
	    	    
	  }

      }

    tracked_object_pub_.publish( tracked_objects );
    object_broadcaster_.sendTransform( object_transforms );
    return;
  }
    
};
    
#endif // USCAUV_OBJECTTRACKING_UNIMODALOBJECTTRACKER
