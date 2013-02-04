/***************************************************************************
 *  include/seabee3_common/markers.h
 *  --------------------
 *
 *  Copyright (c) 2012, Dylan Foster (turtlecannon@gmail.com)
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
 *  * Neither the name of seabee3-ros-pkg nor the names of its
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

#ifndef SEABEE3COMMON_MARKERS_H_
#define SEABEE3COMMON_MARKERS_H_

#include <geometry_msgs/Twist.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_datatypes.h>

// QUICKDEV_DECLARE_X macros
#include <quickdev/macros.h>

// colors
#include <seabee3_common/colors.h>


/** 
 * @param var_name_ Class member variable
 * @param var_name visualization_msgs/Marker member variable
 * @param VarName The function will be called "setVarName"
 */
#define DECLARE_MARKERARRAY_FIELD_MUTATOR3(var_name_, var_name, VarName) \
  void set##VarName( const decltype(var_name_) & value){		\
    for(auto marker_it = markers_.markers.begin(); marker_it != markers_.markers.end(); ++marker_it){ \
      marker_it->var_name = value;} var_name_ = value;}

#define DECLARE_MARKERARRAY_MEMBER_RW3(var_name_, var_name, VarName) \
  QUICKDEV_DECLARE_CONST_ACCESSOR(var_name_, VarName) \
  DECLARE_MARKERARRAY_FIELD_MUTATOR3(var_name_, var_name, VarName)

namespace seabee3_common
{

/// This class only supports homogenous color for constituent markers right now
class CompositeMarker
{
 protected:
  typedef std_msgs::ColorRGBA _ColorRGBA;
  typedef seabee::Color _Color;
  typedef _Color::ColorType _ColorType;
  typedef tf::Point _Point;
  typedef tf::Vector3 _Vector3;
  typedef tf::Quaternion _Quaternion;
  typedef tf::Pose _Pose;
  typedef visualization_msgs::Marker _MarkerMsg;
  typedef visualization_msgs::MarkerArray _MarkerArrayMsg;

 protected:  
  _Color color_;
  _Vector3 scale_;
  _Pose pose_;
  _MarkerArrayMsg markers_;
  
  std::string namespace_;
  int id_offset_;
  int action_;
  ros::Duration lifetime_;
  std::string frame_id_;
  bool frame_locked_;

 public:
 /// TODO: Create more constructors
 CompositeMarker() : 
  color_(_Color(_ColorType::BLACK)),
  scale_(_Vector3(1.0, 1.0, 1.0)),
  pose_(_Pose(_Quaternion(0.0, 0.0, 0.0, 1.0))),
  namespace_(""),
  id_offset_(0),
  action_(0),
  lifetime_(0.2),
  frame_id_(""),
  frame_locked_(false)
  {}
  
  /**
   * Each macro expands to at least one function i.e.:
   * const _Color & getColor() const
   * void setPose(_Pose const &)
   *
   */
  QUICKDEV_DECLARE_CONST_ACCESSOR(color_, Color)
  QUICKDEV_DECLARE_CONST_ACCESSOR(id_offset_, ID)
  
  DECLARE_MARKERARRAY_MEMBER_RW3(namespace_, ns, Namespace)
  DECLARE_MARKERARRAY_MEMBER_RW3(action_, action, Action)
  DECLARE_MARKERARRAY_MEMBER_RW3(lifetime_, lifetime, Lifetime)
  DECLARE_MARKERARRAY_MEMBER_RW3(frame_id_, header.frame_id, FrameID)
  DECLARE_MARKERARRAY_MEMBER_RW3(frame_locked_, frame_locked, FrameLocked)
      
  QUICKDEV_DECLARE_MEMBER_RW2(pose_, Pose)
  QUICKDEV_DECLARE_MEMBER_RW2(scale_, Scale)
	  
  /** 
   * Set the color of every element in the marker array. If an element is a marker type
   * that supports multiple colors, set each of those colors as well. 
   *
   * @param color Color of the marker
   */
  void setColor(_Color const & color)
  {
    color_ = color;
    
    for(auto marker_it = markers_.markers.begin(); marker_it != markers_.markers.end(); ++marker_it)
      {
	
	if( isArrayType( *marker_it ) )
	  {
	    marker_it->colors = std::vector<_ColorRGBA>(marker_it->points.size(), _ColorRGBA(color_));
	  }
	marker_it->color = color_;
      }
    return;
  }

  /** 
   * Set the "id" of the composite marker. The id of each constituent marker is equal to
   * this id plus the index of the marker within the marker array
   * @param id New id for the composite marker
   */
  void setID(int const & id)
  {
    int id_difference = id - id_offset_;
    
    if( !id_difference )
      return;
    
    for(auto marker_it = markers_.markers.begin(); marker_it != markers_.markers.end(); ++marker_it)
      {
	marker_it->id += id_difference;
      }
    
    id_offset_ = id;
    return;
  }
  
  /** 
   * Add a marker to the composite marker. 
   * Only the scale and pose fields are guaranteed to be preserved.
   * Pose of the marker being added is assumed to be expressed with respect to the pose of the composite marker.
   * e.g. if the marker has a position of (0, 0, 0.5), it will appear as offset by 0.5 along the composite marker's z-axis, 
   * regardless of where the composite marker sits in 3-space.
   * @param marker Marker to add
   */  
  void addMarker(_MarkerMsg const & marker)
  {
    _MarkerMsg temp_marker = marker;
    
    temp_marker.ns = namespace_;
    temp_marker.id = markers_.markers.size() + id_offset_;
    temp_marker.action = action_;
    temp_marker.lifetime = lifetime_;
    temp_marker.frame_locked = frame_locked_;
    temp_marker.header.frame_id = frame_id_;
    
    temp_marker.color = color_;

    if( isArrayType(temp_marker))
      {
	temp_marker.colors = std::vector<_ColorRGBA>(temp_marker.points.size(), _ColorRGBA(color_));
      } 
        
    markers_.markers.push_back(temp_marker);
    
    return;
  }  

  /** 
   * Cast our CompositeMarker type to a visualization_msgs/MarkerArray
   * scale field for members of the outgoing array is scaled by scale_ 
   * pose field is offset by pose_
   * @return MarkerArray message corresponding to the composite marker
   */
  operator _MarkerArrayMsg() const
  {
    _MarkerArrayMsg markers = markers_;
    /// auto expands to std::vector<geometry_msgs::Marker>::iterator
    for(auto marker_it = markers.markers.begin(); marker_it != markers.markers.end(); ++marker_it)
      {
	
	/// Express the pose of each marker with respect to the world frame
	_Pose marker_pose;
	tf::poseMsgToTF(marker_it->pose, marker_pose);
	tf::poseTFToMsg(pose_ * marker_pose, marker_it->pose);

	/// TODO: Scale distances from marker origin too
	/// Scale the marker
	marker_it->scale.x *= scale_.x();
	marker_it->scale.y *= scale_.y();
	marker_it->scale.z *= scale_.z();
      }
    
    return markers;
  }

  /** 
   * Reset the pose of the marker to origin with unit rotation
   */
  void resetPose()
  {
    pose_ = _Pose(_Quaternion(0.0, 0.0, 0.0, 1.0));
  }

 private:
    /** 
     * Check whether or not the marker's type is one that supports an array of colors
     * 
     * @param marker geometry_msgs/Marker to check
     * 
     * @return True if the marker is an array type, false otherwise
     */
    inline bool isArrayType(_MarkerMsg const & marker)
    {
      return ( marker.type == _MarkerMsg::LINE_STRIP  ||
	       marker.type == _MarkerMsg::LINE_LIST   ||
	       marker.type == _MarkerMsg::CUBE_LIST   ||
	       marker.type == _MarkerMsg::SPHERE_LIST ||
	       marker.type == _MarkerMsg::POINTS      ||
	       marker.type == _MarkerMsg::TRIANGLE_LIST );
    }
    
};
 

/// Still playing around with this to make a cursor that actually looks good
class CursorMarker : public CompositeMarker
{
 public:
 CursorMarker(): CompositeMarker()
    {
      /* _MarkerMsg z_arrow, yaw_arrow, ring; */

      /* z_arrow.type = yaw_arrow.type = _MarkerMsg::ARROW; */
      /* ring.type = _MarkerMsg::LINE_STRIP; */

      /* /// Populate Z-arrow marker. The marker points along the z axis to the center of the marker at unit rotation. */
      /* /// Quaternion here expresses a 90 degree rotation around the y axis */
      /* tf::poseTFToMsg(_Pose(_Quaternion(0.0, 0.7071, 0.0, 0.7071), */
      /* 			    _Vector3(0.0, 0.0, 0.5)), z_arrow.pose); */
      /* z_arrow.scale.x = z_arrow.scale.y = 2.0; */
      /* z_arrow.scale.z = 0.5; */

      /* /// Populate Yaw marker. The marker points from the center to the ring in the direction of the marker's yaw. */
      /* tf::poseTFToMsg(_Pose(_Quaternion(0.0, 0.0, 0.0, 1.0), */
      /* 			    _Vector3(0.5, 0.0, 0.0)), yaw_arrow.pose); */
      /* yaw_arrow.scale.x = yaw_arrow.scale.y = 2.0; */
      /* yaw_arrow.scale.z = 0.1; */
      
      /* /// Populate ring marker. This marker is an octagonal ring around the marker's pose, sitting on the z-axis when the pose has unit rotation. */
      /* ring.points.resize(9); */
      /* tf::pointTFToMsg(_Point(0.5, 0.0, 0.0), ring.points[0]); */
      /* tf::pointTFToMsg(_Point(0.3536, 0.3536 ,0.0), ring.points[1]); */
      /* tf::pointTFToMsg(_Point(0.0, 0.5, 0.0), ring.points[2]); */
      /* tf::pointTFToMsg(_Point(-0.3536, 0.3536 ,0.0), ring.points[3]); */
      /* tf::pointTFToMsg(_Point(-0.5, 0.0, 0.0), ring.points[4]); */
      /* tf::pointTFToMsg(_Point(-0.3536, -0.3536 ,0.0), ring.points[5]); */
      /* tf::pointTFToMsg(_Point(0.0, -0.5, 0.0), ring.points[6]); */
      /* tf::pointTFToMsg(_Point(0.3536, -0.3536 ,0.0), ring.points[7]); */
      /* tf::pointTFToMsg(_Point(0.5, 0.0, 0.0), ring.points[8]); */
      
      /* /// X component of scale sets line width, others are not used */
      /* tf::vector3TFToMsg(_Vector3(0.1, 0.0, 0.0), ring.scale); */
      /* tf::poseTFToMsg(_Pose(_Quaternion(0.0, 0.0, 0.0, 1.0), */
      /* 			    _Vector3(0.0, 0.0, 0.0)), ring.pose); */
      

      /* /// Add markers */
      /* addMarker(z_arrow); */
      /* addMarker(yaw_arrow); */
      /* addMarker(ring); */

      _MarkerMsg outline;
      outline.type = _MarkerMsg::LINE_STRIP;
      outline.points.resize(5);
      tf::pointTFToMsg(_Point(0.5, 0.0, 0.0), outline.points[0]);
      tf::pointTFToMsg(_Point(-0.25, -0.25 ,0.0), outline.points[1]);
      tf::pointTFToMsg(_Point(0.0, 0.0, 0.0), outline.points[2]);
      tf::pointTFToMsg(_Point(-0.25, 0.25 ,0.0), outline.points[3]);
      tf::pointTFToMsg(_Point(0.5, 0.0, 0.0), outline.points[4]);
      tf::vector3TFToMsg(_Vector3(0.1, 0.0, 0.0), outline.scale);
      tf::poseTFToMsg(_Pose(_Quaternion(0.0, 0.0, 0.0, 1.0),
      			    _Vector3(0.0, 0.0, 0.0)), outline.pose);
      addMarker(outline);
    }
};

} // seabee3_common


namespace seabee
{
  /// TODO: Remove initTwist()
class TrajectoryCursor : public seabee3_common::CursorMarker
{
 public:
  enum Status 
  {
    SELECT,
    CANCEL,
    ACTIVE,
    SUCCESS,
    FAILED
  };

 protected:
  typedef seabee3_common::CursorMarker _CursorMarker;
  typedef geometry_msgs::Twist _TwistMsg;

 public:
  typedef Status StatusType;
  typedef std::map<Status, _Color> StatusColorMap;
  
  const static StatusColorMap status_color_map_;

 protected:
  _CursorMarker cursor_;
  _TwistMsg velocity_;
  StatusType status_;
  ros::Time last_velocity_update_time_;

 public:
 TrajectoryCursor(StatusType status = SELECT)
   : 
  _CursorMarker(), 
  status_(status),
  last_velocity_update_time_(ros::Time::now())
  {
    setColor(status_color_map_.at(status_));
    setNamespace("trajectory_cursor");
    velocity_ = initTwist();
  }
 
 TrajectoryCursor(_TwistMsg velocity, StatusType status = SELECT)
   : 
  _CursorMarker(), 
  velocity_(velocity),
  status_(status),
  last_velocity_update_time_(ros::Time::now())
  {
    setColor(status_color_map_.at(status_));
    setNamespace("trajectory_cursor");
  }
  
  void updatePose()
  {
    /// Get the amount of time since we last updated the pose
    ros::Time now = ros::Time::now();
    double dt = (now - last_velocity_update_time_).toSec();

    /// Integrate linear velocity
    _Vector3 linear_velocity;
    tf::vector3MsgToTF(velocity_.linear, linear_velocity);
    _Vector3 odom_vector = linear_velocity * dt;
    
    /// Integrate angular velocity 
    _Quaternion odom_quat = tf::createQuaternionFromRPY(velocity_.angular.x *dt,
							velocity_.angular.y *dt,
							velocity_.angular.z *dt);
    /// Combine into a single transform
    _Pose odom_pose = _Pose(odom_quat, odom_vector);
    
    /**
     * Compose the two frames. This is equivalent adding the vector of odom_pose 
     * rotated by the quaternion of pose_ to the vector of pose_ and composing the  
     * quaternions of pose_ and odom_pose.
     */
    pose_ *= odom_pose;

    /**
     * Same deal as above, but this doesn't the rotation of the cursor pose to the linear odom vector before adding it
     * The result is that direction controls will make the cursor travel along the X/Y/Z axes of the world frame 
     * instead of the X/Y/Z axes of its own rotated frame
     */
    /* /// Integrate linear velocity and add to current pose */
    /* _Vector3 linear_velocity; */
    /* tf::vector3MsgToTF(velocity_.linear, linear_velocity); */
    /* pose_.setOrigin(pose_.getOrigin() + (linear_velocity * dt)); */

    /* /// Integrate angular velocity and add to current pose */
    /* _Quaternion odom_quat = tf::createQuaternionFromRPY(velocity_.angular.x *dt, */
    /* 		     velocity_.angular.y *dt, */
    /* 		     velocity_.angular.z *dt); */
    /* _Quaternion cursor_pose_quat = pose_.getRotation(); */
    /* /\* pose_.setRotation(cursor_pose_quat * odom_quat); *\/ */
    /* pose_.setRotation(odom_quat * cursor_pose_quat); */

    last_velocity_update_time_ = now;
  }

  QUICKDEV_DECLARE_CONST_ACCESSOR(velocity_, Velocity)
  
    void setVelocity(_TwistMsg const & velocity)
    {
      updatePose();
      velocity_ = velocity;
      return;
    }
  
  /**
   * Expands to a function i.e.:
   * const Status & getStatus() const
   */
  QUICKDEV_DECLARE_CONST_ACCESSOR(status_, Status)

  void setStatus(StatusType const & status)
    {
      status_ = status;
      setColor(status_color_map_.at(status_));
      return;
    }
  
 private:
  _TwistMsg const initTwist(double const & linear_x = 0.0, double const & linear_y = 0.0, 
		      double const & linear_z = 0.0, double const & angular_x = 0.0, 
		      double const & angular_y = 0.0, double const & angular_z = 0.0)
  {
    _TwistMsg twist;
    twist.linear.x = linear_x;
    twist.linear.y = linear_y;
    twist.linear.z = linear_z;
    twist.angular.x = angular_x;
    twist.angular.y = angular_y;
    twist.angular.z = angular_z;
    return twist;
  }
};

/**
 * Aggregate initialization - only available in C++11
 * Note - variable initialization occurs here because it is declared as static
 */
const TrajectoryCursor::StatusColorMap TrajectoryCursor::status_color_map_ = 
  {{Status::SELECT,  Color::ColorType::BLUE},
   {Status::CANCEL,  Color::ColorType::ORANGE},
   {Status::ACTIVE,  Color::ColorType::YELLOW},
   {Status::SUCCESS, Color::ColorType::GREEN},
   {Status::FAILED,  Color::ColorType::RED}};

} // seabee

#endif // SEABEE3COMMON_MARKERS_H_
