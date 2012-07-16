/***************************************************************************
 *  include/seabee3_demo/buoy_task_node.h
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
 *  * Neither the name of usc-ros-pkg nor the names of its
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

#ifndef SEABEE3DEMO_BUOYTASKNODE_H_
#define SEABEE3DEMO_BUOYTASKNODE_H_

#include <quickdev/node.h>

// policies
#include <seabee3_common/seabee_movement_policy.h>
#include <seabee3_common/seabee_recognition_policy.h>

// utils
#include <seabee3_common/recognition_primitives.h>

using namespace seabee;

typedef SeabeeMovementPolicy _SeabeeMovementPolicy;

typedef SeabeeRecognitionPolicy _SeabeeRecognitionPolicy;

QUICKDEV_DECLARE_NODE( BuoyTask, _SeabeeMovementPolicy, _SeabeeRecognitionPolicy )

QUICKDEV_DECLARE_NODE_CLASS( BuoyTask )
{
protected:
    boost::shared_ptr<boost::thread> main_loop_thread_ptr_;

    XmlRpc::XmlRpcValue params_;

    QUICKDEV_DECLARE_NODE_CONSTRUCTOR( BuoyTask )
    {
        //
    }

    QUICKDEV_SPIN_FIRST()
    {
        QUICKDEV_GET_RUNABLE_NODEHANDLE( nh_rel );

        initPolicies<quickdev::policy::ALL>();

        params_ = quickdev::ParamReader::readParam<decltype( params_ )>( nh_rel, "params" );

        main_loop_thread_ptr_ = boost::make_shared<boost::thread>( &BuoyTaskNode::mainLoop, this );
    }

/*
    std::unique_lock<std::mutex> findBuoys( std::set<std::string> const & colors )
    {
        auto landmarks_map_lock = quickdev::make_unique_lock( landmarks_map_mutex_, std::defer_lock );

        findBuoys( colors, landmarks_map_lock );

        return landmarks_map_lock;
    }
*/
/*
    void findBuoys( std::set<std::string> const & colors, std::unique_lock<std::mutex> landmarks_map_lock )
    {
        while( true )
        {
            auto find_buoys_lock = quickdev::make_unique_lock( find_buoys_mutex_ );
            find_buoys_condition_.wait( find_buoys_lock );

            if( !QUICKDEV_GET_RUNABLE_POLICY()::running() ) return;

            landmarks_map_lock.lock();

            std::set<std::string> colors_remaining = colors;

            for( auto landmark_it = landmarks_map_.cbegin(); landmark_it != landmarks_map_.cend(); ++landmark_it )
            {
                auto const & landmark = landmark_it->second;

                std::string const color = landmark.color_;

                if( colors.count( color ) )
                {
                    colors_remaining.erase( color );
                    if( colors_remaining.empty() ) break;
                }
            }

            if( colors_remaining.empty() ) return;
        }
    }
*/

    void alignToBuoy( Landmark const & target )
    {
        auto token = _SeabeeMovementPolicy::moveRelativeTo( target, btTransform( btQuaternion( 0, 0, 0, 1 ), btVector3( 1, 0, 0 ) ) );
        token.wait( 5.0 );
    }

    void boopBuoy( Landmark const & target )
    {
        _SeabeeMovementPolicy::moveAtVelocity( btTransform( btQuaternion( 0, 0, 0, 1 ), btVector3( 0.2, 0, 0 ) ) );
        ros::Duration( 5.0 ).sleep();

        _SeabeeMovementPolicy::moveAtVelocity( btTransform( btQuaternion( 0, 0, 0, 1 ), btVector3( -0.4, 0, 0 ) ) );
        ros::Duration( 5.0 ).sleep();
    }

    void mainLoop()
    {
        // search for and hit first buoy
        while( true )
        {
            Landmark target( Buoy( Color( "orange" ) ) );

            auto find_buoy_token = _SeabeeRecognitionPolicy::findLandmark( target );
            auto rotate_search_token = _SeabeeMovementPolicy::rotateSearch( target, find_buoy_token, Degree( -45 ), Degree( 45.0 ), Degree( 5 ) );

            if( !rotate_search_token.wait( 20 ) ) break;

            alignToBuoy( target );
            boopBuoy( target );

            break;
        }

        // search for and hit second buoy
        while( true )
        {
            Landmark target( Buoy( Color( "green" ) ) );

            auto find_buoy_token = _SeabeeRecognitionPolicy::findLandmark( target );
            auto rotate_search_token = _SeabeeMovementPolicy::rotateSearch( target, find_buoy_token, Radian( Degree( -45 ) ), Radian( Degree( 45.0 ) ), Radian( Degree( 5 ) ) );

            if( !rotate_search_token.wait( 20 ) ) break;

            alignToBuoy( target );
            boopBuoy( target );

            break;
        }
    }

    QUICKDEV_SPIN_ONCE()
    {
        // just update ROS
    }
};

#endif // SEABEE3DEMO_BUOYTASKNODE_H_
