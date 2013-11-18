/***************************************************************************
 *  include/uscauv_visualization/object_visualizer_node.h
 *  --------------------
 *
 *  Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Dylan Foster (turtlecannon@gmail.com)
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


#ifndef USCAUV_USCAUVVISUALIZATION_OBJECTVISUALIZER
#define USCAUV_USCAUVVISUALIZATION_OBJECTVISUALIZER

// ROS
#include <ros/ros.h>

#include <tf/transform_datatypes.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/ColorRGBA.h>

// uscauv
#include <uscauv_common/base_node.h>
#include <uscauv_common/param_loader.h>
#include <uscauv_common/macros.h>
#include <uscauv_common/simple_math.h>

#include <auv_msgs/TrackedObject.h>
#include <auv_msgs/TrackedObjectArray.h>

static double const DEFAULT_ALPHA_RATIO = 0.3;

typedef visualization_msgs::Marker _MarkerMsg;
typedef visualization_msgs::MarkerArray _MarkerArrayMsg;

typedef std_msgs::ColorRGBA _ColorMsg;
typedef std::map<std::string, _ColorMsg> _NamedColorMap;

typedef XmlRpc::XmlRpcValue _XmlVal;

struct ObjectGraphicsStorage
{
  std::string stl_url_;
  _NamedColorMap colors_;
  tf::Transform transform_;
};

typedef std::map<std::string, ObjectGraphicsStorage> _NamedGraphicsMap;
typedef std::map<std::string, XmlRpc::XmlRpcValue> _NamedXmlMap;

/// This was a lot cooler looking in my head
USCAUV_DECLARE_PARAM_LOADER_CONVERSION( ObjectGraphicsStorage, param, 
					
					ObjectGraphicsStorage graphics;
					
					XmlRpc::XmlRpcValue xml_graphics = uscauv::param::lookup<XmlRpc::XmlRpcValue>(param, "graphics");
					XmlRpc::XmlRpcValue colors = uscauv::param::lookup<XmlRpc::XmlRpcValue>(param, "colors");	   
					XmlRpc::XmlRpcValue  transform = uscauv::param::lookup<XmlRpc::XmlRpcValue>(xml_graphics, "transform");
					graphics.stl_url_ = uscauv::param::lookup<std::string>(xml_graphics, "stl_url");

					/// Build a map containing the possible colors that the object can take on and the rgb vals we will use to visualize them
					for( _XmlVal::iterator color_it = colors.begin(); color_it != colors.end(); ++color_it )
					  {
					    _XmlVal visual = uscauv::param::lookup<_XmlVal>( color_it->second, "visual" );
					    _ColorMsg color;
					    
					    /// xmlrpc is finnicky and will throw an exception if we directly cast
					    /// these xmlrpcvalues to doubles if their internal type is int
					    double const & scale = uscauv::param::lookup<double>(visual, "scale");
					    color.r = uscauv::param::lookup<double>(visual, "r")/scale;
					    color.g = uscauv::param::lookup<double>(visual, "g")/scale;
					    color.b = uscauv::param::lookup<double>(visual, "b")/scale;
					    color.a = uscauv::param::lookup<double>(visual, "a")/scale;

					    graphics.colors_.insert( std::make_pair( color_it->first, color ));
					  }

					tf::Quaternion object_to_graphics_quat;
					object_to_graphics_quat.setRPY( uscauv::param::lookup<double>(transform, "roll"),
									uscauv::param::lookup<double>(transform, "pitch"),
									uscauv::param::lookup<double>(transform, "yaw") );
					tf::Vector3 object_to_graphics_vec(uscauv::param::lookup<double>(transform, "x"),
									   uscauv::param::lookup<double>(transform, "y"),
									   uscauv::param::lookup<double>(transform, "z"));
					graphics.transform_ = tf::Transform( object_to_graphics_quat, object_to_graphics_vec );
					
					/* tf::poseTFToMsg( tf::Transform( object_to_graphics_quat, object_to_graphics_vec ), */
					/* 		 graphics.transform_ ); */
					return graphics;
					)

class ObjectVisualizerNode: public BaseNode
{
 private:
  typedef auv_msgs::TrackedObject _TrackedObjectMsg;
  typedef auv_msgs::TrackedObjectArray _TrackedObjectArrayMsg;

  ros::NodeHandle nh_rel_;
  ros::Publisher object_marker_pub_;
  ros::Subscriber tracked_object_sub_;

  _NamedGraphicsMap object_graphics_;
  _MarkerMsg marker_template_;
  _ColorMsg white_template_;

  _TrackedObjectArrayMsg::ConstPtr last_tracked_object_msg_;

  std::string object_ns_;

  double estimate_alpha_ratio_;
  
 public:
 ObjectVisualizerNode(): BaseNode("ObjectVisualizer"), nh_rel_("~")
    {
      object_ns_ = "model/objects";
    }

 private:

  // Running spin() will cause this function to be called before the node begins looping the spinOnce() function.
  void spinFirst()
  {
    ros::NodeHandle nh_base;
    /* XmlRpc::XmlRpcValue xml_objects; */
       
    object_marker_pub_ = nh_rel_.advertise<_MarkerArrayMsg>("markers", 1 );
    tracked_object_sub_ = nh_base.subscribe("robot/sensors/tracked_objects", 10,
					    &ObjectVisualizerNode::trackedObjectArrayCallback, this );
       
    // ################################################################
    // Load objects definitions from parameter server #################
    // ################################################################
       
    /// fancy one liner thanks to tons of code in param_loader.h
    object_graphics_ = uscauv::param::load<_NamedGraphicsMap>( nh_base, object_ns_ );
       
    estimate_alpha_ratio_ = uscauv::param::load<double>( nh_rel_, "alpha_ratio", DEFAULT_ALPHA_RATIO);
    if( !uscauv::in_range_closed(0.0, 1.0, estimate_alpha_ratio_ ) )
      {
	ROS_WARN_STREAM( "Received alpha ratio " << brk(estimate_alpha_ratio_) << ", but valid range is [0, 1]. Defaulting to " << brk(DEFAULT_ALPHA_RATIO) );
	estimate_alpha_ratio_ = DEFAULT_ALPHA_RATIO;
      }
       
    /// Init params that will be used across all markers
    marker_template_.type = visualization_msgs::Marker::MESH_RESOURCE;
    marker_template_.action = visualization_msgs::Marker::ADD;
    /* tf::poseTFToMsg( tf::Transform::getIdentity(), marker_template_.pose ); */
    marker_template_.scale.x =
      marker_template_.scale.y =
      marker_template_.scale.z = 1;

       
    marker_template_.lifetime = ros::Duration( 2 / getLoopRate() ); /// arbitrary time that shouldn't be too much longer than loop period
    marker_template_.frame_locked = true;
    marker_template_.mesh_use_embedded_materials = false;

    white_template_.r = 
      white_template_.g = 
      white_template_.b = 
      white_template_.a = 1.0;
  }  

  // Running spin() will cause this function to get called at the loop rate until this node is killed.
  void spinOnce()
  {
    if( !last_tracked_object_msg_)
      return;
	
    _MarkerArrayMsg markers;
    int idx = 0;
    for( _TrackedObjectMsg const & object: last_tracked_object_msg_->objects )
      {
	_NamedGraphicsMap::const_iterator graphics_it = object_graphics_.find( object.type );
	if( graphics_it == object_graphics_.end() )
	  {
	    /// only print this error message every 30 seconds
	    ROS_WARN_STREAM_THROTTLE(30, "Received object " << brk( object.type ) << ", but graphics data for this object could not be found." );
	    continue;
	  }

	
	_MarkerMsg marker = marker_template_;
	marker.header = object.header;
	marker.mesh_resource = graphics_it->second.stl_url_;
	marker.ns = object.type;
	marker.id = idx; /// each object type should probably get its own count but it doesn't really matter

	/// Try to find RGB data for the current object color and set it to white if we can't find anything
	_NamedColorMap::const_iterator color_it = graphics_it->second.colors_.find( object.color );
	if( color_it == graphics_it->second.colors_.end() )
	  {
	    ROS_WARN_STREAM_THROTTLE(30, "Received object " << brk( object.type ) << ", in color state " << brk( object.color ) << " but graphics for this color could not be found." );
	    marker.color = white_template_;
	  }
	else
	  {
	    marker.color = color_it->second;
	  }

	/// reduce object transparency if it isn't the best estimate for its type
	if( !object.is_best_estimate )
	  marker.color.a *= estimate_alpha_ratio_;
	   
	/// Set transform from object cad model to the motion frame (tf will take care of transforming from this to the world frame
	tf::Transform object_to_motion_tf;
	tf::poseMsgToTF( object.pose.pose, object_to_motion_tf );
	tf::poseTFToMsg( object_to_motion_tf * graphics_it->second.transform_, marker.pose );
	
	markers.markers.push_back( marker );
	   
	++idx;
      }
    object_marker_pub_.publish( markers );
  }


  void trackedObjectArrayCallback( _TrackedObjectArrayMsg::ConstPtr const & msg )
  {
    last_tracked_object_msg_ = msg;
  }

};

#endif // USCAUV_USCAUVVISUALIZATION_OBJECTVISUALIZER
