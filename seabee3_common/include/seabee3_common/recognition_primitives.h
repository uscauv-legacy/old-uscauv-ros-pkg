/***************************************************************************
 *  include/seabee3_common/recognition_primitives.h
 *  --------------------
 *
 *  Copyright (c) 2011, Edward T. Kaszubski ( ekaszubski@gmail.com )
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

#ifndef SEABEE3COMMON_RECOGNITIONPRIMITIVES_H_
#define SEABEE3COMMON_RECOGNITIONPRIMITIVES_H_

// objects
#include <map>
#include <quickdev/action_token.h>
#include <seabee3_common/colors.h>
#include <seabee3_common/motion_primitives.h>
#include <image_geometry/pinhole_camera_model.h>

// utils
#include <quickdev/math.h>

// msgs
#include <seabee3_msgs/LandmarkArray.h>
#include <visualization_msgs/MarkerArray.h>

namespace seabee
{

    typedef seabee3_msgs::Landmark _LandmarkMsg;
    typedef seabee3_msgs::LandmarkArray _LandmarkArrayMsg;

    typedef visualization_msgs::Marker _MarkerMsg;
    typedef visualization_msgs::MarkerArray _MarkerArrayMsg;

    typedef image_geometry::PinholeCameraModel _PinholeCameraModel;

// =============================================================================================================================================
class Landmark
{
public:
    Pose pose_;
    Size size_;

    enum LandmarkType
    {
        GATE = 0,
        BUOY,
        PIPE,
        HEDGE,
        WINDOW,
        BIN,
        PINGER
    };

    LandmarkType type_;
    Color color_;
    std::string name_;

    Landmark()
    {
        //
    }

    Landmark( Landmark const & other )
    :
        pose_( other.pose_ ),
        size_( other.size_ ),
        type_( other.type_ ),
        color_( other.color_ ),
        name_( other.name_ )
    {
        //
    }

    Landmark( _LandmarkMsg const & landmark_msg )
    :
        pose_( unit::convert<Pose>( landmark_msg.pose ) ),
        size_( unit::convert<Size>( landmark_msg.size ) ),
        type_( LandmarkType( landmark_msg.type ) ),
        color_( landmark_msg.color ),
        name_( getUniqueName() )
    {
        //
    }

    template
    <
        class... __Args,
        typename std::enable_if<(!sizeof...(__Args) == 1 || !boost::is_base_of<
            Landmark,
            typename std::remove_reference<
                typename std::remove_const<
                    typename variadic::element<0, __Args...>::type
                >::type
            >::type
        >::value), int>::type = 0
    >
    Landmark( __Args&&... args )
    {
        init( args... );
        name_ = getUniqueName();
    }

    // ##[ Initialization ]#####################################################################################################################
    template<class... __Args>
    void init( LandmarkType const & type, __Args&&... args )
    {
        type_ = type;
        init( args... );
    }

    template<class... __Args>
    void init( Pose const & pose, __Args&&... args )
    {
        pose_ = pose;
        init( args... );
    }

    template<class... __Args>
    void init( Size const & size, __Args&&... args )
    {
        size_ = size;
        init( args... );
    }

    template<class... __Args>
    void init( Color const & color, __Args&&... args )
    {
        color_ = color;
        init( args... );
    }

    void init() const {}

    bool operator<( Landmark const & other ) const
    {
        return std::min( size_.x_, size_.y_ ) < std::min( other.size_.x_, other.size_.y_ );
    }

    bool operator==( Landmark const & other ) const
    {
        return name_ == other.name_;
    }

    operator _LandmarkMsg() const
    {
        _LandmarkMsg landmark_msg;
        landmark_msg.type = type_;
        landmark_msg.color = color_;
        landmark_msg.pose = unit::implicit_convert( pose_ );
        landmark_msg.size = unit::implicit_convert( size_ );
        landmark_msg.name = name_.empty() ? getUniqueName() : name_;

        return landmark_msg;
    }

    operator _MarkerMsg() const
    {
        _MarkerMsg marker_msg;
        marker_msg.header.stamp = ros::Time::now();

        marker_msg.color = color_;

        marker_msg.ns = "landmarks";
        marker_msg.action = visualization_msgs::Marker::ADD;
        marker_msg.lifetime = ros::Duration( 0.1 );

        marker_msg.pose.position = unit::implicit_convert( pose_.position_ );

        if( type_ == PIPE || type_ == BIN ) marker_msg.header.frame_id = "/seabee3/camera2";
        else marker_msg.header.frame_id = "/seabee3/camera1";

        switch( type_ )
        {
        case BUOY:
            marker_msg.type = visualization_msgs::Marker::SPHERE;
            marker_msg.scale.x =
            marker_msg.scale.y =
            marker_msg.scale.z = size_.x_ + size_.y_ / 2;
            break;
        case PIPE:
            marker_msg.type = visualization_msgs::Marker::ARROW;
            marker_msg.scale.x = size_.x_;
            marker_msg.scale.y = size_.y_;
            marker_msg.scale.z = size_.z_;
            marker_msg.pose.orientation = unit::implicit_convert( btQuaternion( 0, -M_PI_2, 0 ) * btQuaternion( pose_.orientation_.yaw_ + M_PI_2, 0, 0 ) );
            break;
        }

        return marker_msg;
    }

    operator std::string() const
    {
        return name_;
    }

    std::string getUniqueName() const
    {
        return getTypeName() + "_" + std::string( color_ );
    }

    //! Get real object diameter in meters
    Size getIdealSize() const
    {
        switch( type_ )
        {
        case BUOY:
            return Size( 0.2032, 0.2032, 0.2032 );
        case PIPE:
            return Size( 0.6, 0.1, 0.001 );
            //return Size( 1.2192, 0.1, 0.001 );
            //return Size( 0.4064, 0.3, 0.3 );
        }
        return 0;
    }

    std::string getTypeName() const
    {
        switch( type_ )
        {
        case BUOY:
            return "buoy";
        case PIPE:
            return "pipe";
        }
        return "";
    }

    void projectTo3d( _PinholeCameraModel const & camera_model )
    {
        cv::Point2d const center_point( pose_.position_.x_, pose_.position_.y_ );
        // ray to center of landmark
        cv::Point3d const center_ray = camera_model.projectPixelTo3dRay( center_point );
        btVector3 const center_unit_vec( center_ray.x, center_ray.y, center_ray.z );

        PRINT_INFO( "center pixel: %f %f", center_point.x, center_point.y );
        PRINT_INFO( "center ray: %f %f %f", center_unit_vec.getX(), center_unit_vec.getY(), center_unit_vec.getZ() );

        // get radius (center to widest edge) of object
        auto const size_meters = getIdealSize();
        // we always use the max length as a diameter
        auto const size_pixels = size_;

        double const radius_meters = quickdev::min( size_meters.x_, size_meters.y_ ) / 2.0;
        double const radius_pixels = quickdev::min( size_pixels.x_, size_pixels.y_ ) / 2.0;

        cv::Point2d const radius_point( pose_.position_.x_ + radius_pixels, pose_.position_.y_ );

        // get the ray to a point on the radius of the object
        cv::Point3d const radius_ray = camera_model.projectPixelTo3dRay( radius_point );
        btVector3 const radius_unit_vec( radius_ray.x, radius_ray.y, radius_ray.z );

        PRINT_INFO( "radius pixel: %f %f", radius_point.x, radius_point.y );
        PRINT_INFO( "radius ray: %f %f %f", radius_unit_vec.getX(), radius_unit_vec.getY(), radius_unit_vec.getZ() );

        // get the angle between the center and radius rays
        double angle = center_unit_vec.angle( radius_unit_vec );

        // get the distance to the object
        double distance = radius_meters / tan( angle );

        // project the center ray out to the distance calculated
        btVector3 center_vec = center_unit_vec * distance;

        // update the position of the landmark given the newly projected center location
        pose_.position_.x_ = center_vec.getZ();
        pose_.position_.y_ = -center_vec.getX();
        pose_.position_.z_ = -center_vec.getY();

        size_ = size_meters;
    }
};

// =============================================================================================================================================
class Buoy : public Landmark
{
public:
    template<class... __Args>
    Buoy( __Args&&... args )
    :
        Landmark( Landmark::BUOY, args... )
    {
        //
    }


};

// =============================================================================================================================================
class Pipe : public Landmark
{
public:
    template<class... __Args>
    Pipe( __Args&&... args )
    :
        Landmark( Landmark::PIPE, Color( Color::ORANGE ), args... )
    {
        //
    }
};

} // seabee

#endif // SEABEE3COMMON_RECOGNITIONPRIMITIVES_H_
