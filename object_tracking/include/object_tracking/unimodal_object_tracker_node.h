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

// general uscauv
#include <uscauv_common/base_node.h>
#include <uscauv_common/multi_reconfigure.h>
#include <uscauv_common/param_loader.h>
#include <auv_msgs/MatchedShape.h>
#include <auv_msgs/MatchedShapeArray.h>

/// object tracking
#include <object_tracking/kalman_filter.h>
#include <object_tracking/TrackedObjectConfig.h>

#include <sensor_msgs/CameraInfo.h>

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
  double ideal_radius_;
  bool tracked_;
  std::string name_;
};

typedef std::map<std::string, std::string>          _NamedAttributeMap;
typedef std::map<std::string, ObjectTrackerStorage> _AttributeTrackerMap;

class UnimodalObjectTrackerNode: public BaseNode, public MultiReconfigure
{
 private:
  /// ros
  ros::NodeHandle nh_rel_;
  ros::Subscriber matched_shape_sub_, camera_info_sub_;
  
  std::string const object_ns;

  /// algorithmic
  _NamedAttributeMap name_size_color_map_;
  _AttributeTrackerMap trackers_;

  /// other
  _CameraInfo last_camera_info_;
  
 public:
 UnimodalObjectTrackerNode(): BaseNode("UnimodalObjectTracker"), 
    MultiReconfigure( ros::NodeHandle("model/objects") ), /// resolves below node namespaces
    nh_rel_("~"), object_ns("model/objects")
    {
    }

  void matchedShapeCallback( _MatchedShapeArray::ConstPtr const & msg )
  {
    for( std::vector<_MatchedShape>::const_iterator shape_it= msg->shapes.begin();
	 shape_it != msg->shapes.end(); ++shape_it)
      {
	/// TODO: Something interesting here 
      }
  }

  /// cache camera info
  void cameraInfoCallback( _CameraInfo::ConstPtr const & msg )
  {
    last_camera_info_ = *msg; 
  }

  void updateTrackerParams(_TrackedObjectConfig const & config, std::string const & name)
  {
    double const & var = config.predict_variance;
    _ObjectKalmanFilter::ControlMatrix control_cov;
    control_cov <<
      var, 0, 0, 0,
      0, var, 0, 0,
      0, 0, var, 0,
      0, 0, 0, var;
    trackers_.at( name_size_color_map_.at( name ) ).control_cov_ = control_cov;
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
       
    // ################################################################
    // Load objects definitions from parameter server #################
    // ################################################################
    xml_objects = uscauv::loadParam<uscauv::XmlRpcValue>( nh_base, object_ns );
       
    for( _NamedXmlMap::iterator object_it = xml_objects.begin(); 
	 object_it != xml_objects.end(); ++object_it )
      {
	std::string const attr = std::string(object_it->second["shape"]) + "/" +
	  std::string(object_it->second["color"]);
	name_size_color_map_[ object_it->first] = attr;
	   
	ObjectTrackerStorage tracker;
	tracker.name_ = object_it->first;
	tracker.ideal_radius_ = object_it->second["ideal_radius"];
	tracker.tracked_ = (immediate_tracking) ? true : false;
	
	/// have to add the new object before or the lookup in the reconfigure callback will fail to find it
	trackers_[ attr ] = tracker;
	   
	/// set up reconfigure
	addReconfigureServer<_TrackedObjectConfig>
	  ( object_it->first, std::bind( &UnimodalObjectTrackerNode::updateTrackerParams, this,
	   				 std::placeholders::_1, object_it->first ) );
	   
	ROS_INFO("Loaded object [ %s ] with attributes [ %s ].", 
		 object_it->first.c_str(), attr.c_str() );
      }
       
       
  }  

  // Running spin() will cause this function to get called at the loop rate until this node is killed.
  void spinOnce()
  {
    /// TODO: Publish tracked objects

  }

};

#endif // USCAUV_OBJECTTRACKING_UNIMODALOBJECTTRACKER
