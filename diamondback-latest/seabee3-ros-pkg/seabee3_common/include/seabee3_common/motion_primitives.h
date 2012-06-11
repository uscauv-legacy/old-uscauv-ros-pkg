/***************************************************************************
 *  include/seabee3_common/motion_primitives.h
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

#ifndef SEABEE3COMMON_MOTIONPRIMITIVES_H_
#define SEABEE3COMMON_MOTIONPRIMITIVES_H_

#include <quickdev/action_token.h>
#include <quickdev/geometry_message_conversions.h>
#include <geometry_msgs/Pose.h>

namespace seabee
{

// =============================================================================================================================================
//! A 3D position
class Position
{
public:
    double x_;
    double y_;
    double z_;

    Position( double const & x = 0, double const & y = 0, double const & z = 0 )
    :
        x_( x ),
        y_( y ),
        z_( z )
    {
        //
    }

    // ##[ Operators ]##########################################################################################################################
    Position operator+( Position const & other ) const
    {
        return Position
        (
            x_ + other.x_,
            y_ + other.y_,
            z_ + other.z_
        );
    }

    Position & operator+=( Position const & other )
    {
        x_ += other.x_;
        y_ += other.y_;
        z_ += other.z_;

        return *this;
    }
};

// =============================================================================================================================================
//! An orientation
class Orientation
{
public:
    double yaw_;

    Orientation( double const & yaw = 0 )
    :
        yaw_( yaw )
    {
        //
    }

    // ##[ Operators ]##########################################################################################################################
    Orientation operator+( Orientation const & other ) const
    {
        return Orientation( yaw_ + other.yaw_ );
    }

    Orientation & operator+=( Orientation const & other )
    {
        yaw_ += other.yaw_;
        return *this;
    }
};

// =============================================================================================================================================
//! The combined Position and Orientation of a Landmark
class Pose
{
public:
    Position position_;
    Orientation orientation_;

    Pose()
    {
        //
    }

    template
    <
        class... __Args,
        typename std::enable_if<(!sizeof...(__Args) == 1 || !std::is_same<
            typename std::remove_reference<
                typename std::remove_const<
                    typename variadic::element<0, __Args...>::type
                >::type
            >::type,
            Pose
        >::value), int>::type = 0
    >
    Pose( __Args&&... args )
    {
        init( args... );
    }

    /*Pose( Pose & other )
    {
        if( this != &other ) this = &other;
    }*/

    // ##[ Operators ]##########################################################################################################################
    Pose operator+( Position const & other ) const
    {
        return Pose( position_ + other );
    }

    Pose & operator+=( Position const & other )
    {
        position_ += other;

        return *this;
    }

    Pose operator+( Orientation const & other ) const
    {
        return Pose( orientation_ + other );
    }

    Pose & operator+=( Orientation const & other )
    {
        orientation_ += other;

        return *this;
    }

    Pose operator+( Pose const & other ) const
    {
        return Pose( position_ + other.position_, orientation_ + other.orientation_ );
    }

    Pose & operator+=( Pose const & other )
    {
        operator+=( other.position_ );
        operator+=( other.orientation_ );

        return *this;
    }

    // ##[ Initialization ]#####################################################################################################################
    template<class... __Args>
    void init( Position const & position, __Args&&... args )
    {
        position_ = position;
        init( args... );
    }

    template<class... __Args>
    void init( Orientation const & orientation, __Args&&... args )
    {
        orientation_ = orientation;
        init( args... );
    }

    void init() const {}
};

// =============================================================================================================================================
class FiringDevice
{
public:
    struct Type
    {
        static const int Shooter = 0;
        static const int Dropper = 1;
    };

    template<int __Type__, class __Id>
    static int getDeviceId( __Id const & id )
    {
        return __Type__ * __Id::size + id.value_;
    }

    int device_id_;

    FiringDevice( int const & device_id )
    :
        device_id_( device_id )
    {
        //
    }
};

// =============================================================================================================================================
class Shooter : public FiringDevice
{
public:
    struct Id
    {
        static const int Shooter1 = 0;
        static const int Shooter2 = 1;

        static const int size = 2;

        int value_;

        Id( int const & value ) : value_( value ){}
    };

    Shooter( Id const & id )
    :
        FiringDevice( getDeviceId<FiringDevice::Type::Shooter>( id ) )
    {
        //
    }
};

// =============================================================================================================================================
class Dropper : public FiringDevice
{
public:
    struct Id
    {
        static const int Dropper1 = 0;
        static const int Dropper2 = 1;

        static const int size = 2;

        int value_;

        Id( int const & value ) : value_( value ){}
    };

    Dropper( Id const & id )
    :
        FiringDevice( getDeviceId<FiringDevice::Type::Dropper>( id ) )
    {
        //
    }
};

} // seabee

// seabee::Position <-> geometry_msgs::Point
DECLARE_UNIT_CONVERSION_LAMBDA( seabee::Position, geometry_msgs::Point, pos, geometry_msgs::Point point; point.x = pos.x_; point.y = pos.y_; point.z = pos.z_; return point; )
DECLARE_UNIT_CONVERSION_LAMBDA( geometry_msgs::Point, seabee::Position, point, return seabee::Position( point.x, point.y, point.z ); )
// seabee::Position <-> geometry_msgs::Vector3
DECLARE_UNIT_CONVERSION_LAMBDA( seabee::Position, geometry_msgs::Vector3, pos, geometry_msgs::Vector3 vec; vec.x = pos.x_; vec.y = pos.y_; vec.z = pos.z_; return vec; )
DECLARE_UNIT_CONVERSION_LAMBDA( geometry_msgs::Vector3, seabee::Position, vec, return seabee::Position( vec.x, vec.y, vec.z ); )
// seabee::Position <-> btVector3
DECLARE_UNIT_CONVERSION_LAMBDA( seabee::Position, btVector3, pos, return btVector3( pos.x_, pos.y_, pos.z_ ); )
DECLARE_UNIT_CONVERSION_LAMBDA( btVector3, seabee::Position, vec, return seabee::Position( vec.getX(), vec.getY(), vec.getZ() ); )


// seabee::Orientation <-> geometry_msgs::Vector3
DECLARE_UNIT_CONVERSION_LAMBDA( seabee::Orientation, geometry_msgs::Vector3, ori, geometry_msgs::Vector3 vec; vec.z = ori.yaw_; return vec; )
DECLARE_UNIT_CONVERSION_LAMBDA( geometry_msgs::Vector3, seabee::Orientation, vec, return seabee::Orientation( vec.z ); )
// seabee::Orientation <-> btVector3
DECLARE_UNIT_CONVERSION_LAMBDA( seabee::Orientation, btVector3, ori, btVector3 vec; vec.z = ori.yaw_; return unit::convert<btVector3>( vec ); )
DECLARE_UNIT_CONVERSION_LAMBDA( btVector3, seabee::Orientation, vec, return seabee::Orientation( vec.getZ() ); )
// seabee::Orientation <-> geometry_msgs::Quaternion
DECLARE_UNIT_CONVERSION_LAMBDA( seabee::Orientation, geometry_msgs::Quaternion, ori, return unit::convert<geometry_msgs::Quaternion>( unit::convert<geometry_msgs::Vector3>( ori ) ); )
DECLARE_UNIT_CONVERSION_LAMBDA( geometry_msgs::Quaternion, seabee::Orientation, quat, return unit::convert<seabee::Orientation>( unit::convert<geometry_msgs::Vector3>( quat ) ); )
// seabee::Orientation <-> btQuaternion
DECLARE_UNIT_CONVERSION_LAMBDA( seabee::Orientation, btQuaternion, ori, return unit::convert<btQuaternion>( unit::convert<geometry_msgs::Vector3>( ori ) ); )
DECLARE_UNIT_CONVERSION_LAMBDA( btQuaternion, seabee::Orientation, quat, return unit::convert<seabee::Orientation>( unit::convert<geometry_msgs::Vector3>( quat ) ); )


// seabee::Pose <-> geometry_msgs::Twist
DECLARE_UNIT_CONVERSION_LAMBDA( seabee::Pose, geometry_msgs::Twist, pose, geometry_msgs::Twist twist; twist.linear = unit::make_unit( pose.position_ ); twist.angular = unit::make_unit( pose.orientation_ ); return twist; )
DECLARE_UNIT_CONVERSION_LAMBDA( geometry_msgs::Twist, seabee::Pose, twist, return seabee::Pose( unit::convert<seabee::Position>( twist.linear ), unit::convert<seabee::Orientation>( twist.angular ) ); )
// seabee::Pose <-> btTransform
DECLARE_UNIT_CONVERSION_LAMBDA( seabee::Pose, btTransform, pose, return btTransform( unit::make_unit( pose.orientation_ ), unit::make_unit( pose.position_ ) ); )
DECLARE_UNIT_CONVERSION_LAMBDA( btTransform, seabee::Pose, tf, return seabee::Pose( unit::convert<seabee::Position>( tf.getOrigin() ), unit::convert<seabee::Orientation>( tf.getRotation() ) ); )
// seabee::Pose <-> geometry_msgs::Pose
DECLARE_UNIT_CONVERSION_LAMBDA( seabee::Pose, geometry_msgs::Pose, pose, geometry_msgs::Pose pose_msg; pose_msg.position = unit::make_unit( pose.position_ ); pose_msg.orientation = unit::make_unit( pose.orientation_ ); return pose_msg; )
DECLARE_UNIT_CONVERSION_LAMBDA( geometry_msgs::Pose, seabee::Pose, pose_msg, return seabee::Pose( unit::convert<seabee::Position>( pose_msg.position ), unit::convert<seabee::Orientation>( pose_msg.orientation ) ); )

#endif // SEABEE3COMMON_MOTIONPRIMITIVES_H_
