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

typedef visualization_msgs::Marker _MarkerMsg;
typedef visualization_msgs::MarkerArray _MarkerArrayMsg;


struct ObjectGraphicsStorage
{
  std::string stl_url_;
  std_msgs::ColorRGBA color_;
  geometry_msgs::Pose transform_;
};

/// This was a lot cooler looking in my head
USCAUV_DECLARE_PARAM_LOADER_CONVERSION( ObjectGraphicsStorage, param, 
					
					ObjectGraphicsStorage graphics;
					
					XmlRpc::XmlRpcValue xml_graphics = uscauv::param::lookup<XmlRpc::XmlRpcValue>(param, "graphics");
					XmlRpc::XmlRpcValue color = uscauv::param::lookup<XmlRpc::XmlRpcValue>(xml_graphics, "color");	   
					XmlRpc::XmlRpcValue  transform = uscauv::param::lookup<XmlRpc::XmlRpcValue>(xml_graphics, "transform");
					graphics.stl_url_ = uscauv::param::lookup<std::string>(xml_graphics, "stl_url");
					/// xmlrpc is finnicky and will throw an exception if we directly cast
					/// these xmlrpcvalues to doubles if their internal type is int
					double const & scale = uscauv::param::lookup<double>(color, "scale");
					graphics.color_.r = uscauv::param::lookup<double>(color, "r")/scale;
					graphics.color_.g = uscauv::param::lookup<double>(color, "g")/scale;
					graphics.color_.b = uscauv::param::lookup<double>(color, "b")/scale;
					graphics.color_.a = uscauv::param::lookup<double>(color, "a")/scale;
					
					tf::Quaternion object_to_graphics_quat;
					object_to_graphics_quat.setRPY( uscauv::param::lookup<double>(transform, "roll"),
									uscauv::param::lookup<double>(transform, "pitch"),
									uscauv::param::lookup<double>(transform, "yaw") );
					tf::Vector3 object_to_graphics_vec(uscauv::param::lookup<double>(transform, "x"),
									   uscauv::param::lookup<double>(transform, "y"),
									   uscauv::param::lookup<double>(transform, "z"));
					tf::poseTFToMsg( tf::Transform( object_to_graphics_quat, object_to_graphics_vec ),
							 graphics.transform_ );
					return graphics;
					)

typedef std::map<std::string, ObjectGraphicsStorage> _NamedGraphicsMap;
typedef std::map<std::string, XmlRpc::XmlRpcValue> _NamedXmlMap;

class ObjectVisualizerNode: public BaseNode
{
 private:
  ros::NodeHandle nh_rel_;
  ros::Publisher object_marker_pub_;

  _NamedGraphicsMap object_graphics_;
  _MarkerMsg marker_template_;

  std::string object_ns_;
  
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
       
       // ################################################################
       // Load objects definitions from parameter server #################
       // ################################################################
       
       /// fancy one liner thanks to tons of code in param_loader.h
       object_graphics_ = uscauv::param::load<_NamedGraphicsMap>( nh_base, object_ns_ );
       
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
     }  

  // Running spin() will cause this function to get called at the loop rate until this node is killed.
  void spinOnce()
     {
       _MarkerArrayMsg markers;
       
       for( _NamedGraphicsMap::const_iterator graphics_it = object_graphics_.begin();
	    graphics_it != object_graphics_.end(); ++graphics_it )
	 {
	   _MarkerMsg marker = marker_template_;
	   marker.ns = graphics_it->first;
	   marker.mesh_resource = graphics_it->second.stl_url_;
	   marker.color = graphics_it->second.color_;
	   marker.pose = graphics_it->second.transform_;

	   marker.header.frame_id = std::string( "object/" + graphics_it->first);
	   marker.header.stamp = ros::Time::now();
   
	   markers.markers.push_back( marker );
	 }
       
       object_marker_pub_.publish( markers );
     }

};

#endif // USCAUV_USCAUVVISUALIZATION_OBJECTVISUALIZER
