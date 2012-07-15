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

// msgs
#include <seabee3_msgs/LandmarkArray.h>
#include <visualization_msgs/MarkerArray.h>

namespace seabee
{

    typedef seabee3_msgs::Landmark _LandmarkMsg;
    typedef seabee3_msgs::LandmarkArray _LandmarkArrayMsg;

    typedef visualization_msgs::Marker _MarkerMsg;
    typedef visualization_msgs::MarkerArray _MarkerArrayMsg;

// =============================================================================================================================================
class Landmark
{
public:
    Pose pose_;
    Size size_;

    enum LandmarkType
    {
        GATE,
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

    template
    <
        class... __Args,
        typename std::enable_if<(!sizeof...(__Args) == 1 || !std::is_same<
            typename std::remove_reference<
                typename std::remove_const<
                    typename variadic::element<0, __Args...>::type
                >::type
            >::type,
            Landmark
        >::value), int>::type = 0
    >
    Landmark( __Args&&... args )
    {
        init( args... );
    }

    // ##[ Initialization ]#####################################################################################################################
    template<class... __Args>
    void init( std::string const & name, __Args&&... args )
    {
        name_ = name;
        init( args... );
    }

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

    operator _LandmarkMsg() const
    {
        _LandmarkMsg landmark_msg;
        landmark_msg.type = type_;
        landmark_msg.color = color_;
        landmark_msg.pose = unit::make_unit( pose_ );
        landmark_msg.size = unit::make_unit( size_ );

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

        marker_msg.pose = unit::make_unit( pose_ );

        if( type_ == PIPE || type_ == BIN ) marker_msg.header.frame_id = "camera2";
        else marker_msg.header.frame_id = "camera1";

        switch( type_ )
        {
        case BUOY:
            marker_msg.type = visualization_msgs::Marker::SPHERE;
            marker_msg.scale.x =
            marker_msg.scale.y = size_.x_ + size_.y_ / 2;
            break;
        case PIPE:
            marker_msg.type = visualization_msgs::Marker::CUBE;
            marker_msg.scale.x = size_.x_;
            marker_msg.scale.y = size_.y_;
            marker_msg.scale.z = 0.005;
            break;
        }

        return marker_msg;
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
        Landmark( Landmark::PIPE, Color::ORANGE, args... )
    {
        //
    }
};

} // seabee

#endif // SEABEE3COMMON_RECOGNITIONPRIMITIVES_H_
