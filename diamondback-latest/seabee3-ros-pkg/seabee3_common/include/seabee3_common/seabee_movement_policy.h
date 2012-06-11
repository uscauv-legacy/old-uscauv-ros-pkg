/***************************************************************************
 *  include/seabee3_common/seabee_movement_policy.h
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

#ifndef SEABEE3COMMON_SEABEEMOVEMENTPOLICY_H_
#define SEABEE3COMMON_SEABEEMOVEMENTPOLICY_H_

// policies
#include <quickdev/tf_tranceiver_policy.h>
#include <quickdev/action_client_policy.h>

// objects
#include <seabee3_common/motion_primitives.h>
#include <quickdev/multi_subscriber.h>

// actions
#include <seabee3_actions/MakeTrajectoryAction.h>
#include <seabee3_actions/FollowTrajectoryAction.h>

// msgs
#include <seabee3_msgs/TrajectoryWaypoint.h>

using namespace seabee;

QUICKDEV_DECLARE_POLICY_NS( SeabeeMovement )
{
    typedef seabee3_msgs::TrajectoryWaypoint _TrajectoryWaypointMsg;

    typedef seabee3_actions::MakeTrajectoryAction _MakeTrajectoryAction;
    typedef seabee3_actions::FollowTrajectoryAction _FollowTrajectoryAction;

    typedef quickdev::TfTranceiverPolicy _TfTranceiverPolicy;
    typedef quickdev::ActionClientPolicy<_MakeTrajectoryAction> _MakeTrajectoryActionClientPolicy;
    typedef quickdev::ActionClientPolicy<_FollowTrajectoryAction> _FollowTrajectoryActionClientPolicy;
}

QUICKDEV_DECLARE_POLICY( SeabeeMovement, _TfTranceiverPolicy, _MakeTrajectoryActionClientPolicy, _FollowTrajectoryActionClientPolicy )

QUICKDEV_DECLARE_POLICY_CLASS( SeabeeMovement )
{
public:
    typedef QUICKDEV_GET_POLICY_NS( SeabeeMovement )::_TrajectoryWaypointMsg _TrajectoryWaypointMsg;

    typedef QUICKDEV_GET_POLICY_NS( SeabeeMovement )::_MakeTrajectoryAction _MakeTrajectoryAction;
    typedef QUICKDEV_GET_POLICY_NS( SeabeeMovement )::_FollowTrajectoryAction _FollowTrajectoryAction;

    typedef QUICKDEV_GET_POLICY_NS( SeabeeMovement )::_TfTranceiverPolicy _TfTranceiverPolicy;
    typedef QUICKDEV_GET_POLICY_NS( SeabeeMovement )::_MakeTrajectoryActionClientPolicy _MakeTrajectoryActionClientPolicy;
    typedef QUICKDEV_GET_POLICY_NS( SeabeeMovement )::_FollowTrajectoryActionClientPolicy _FollowTrajectoryActionClientPolicy;

    typedef _TrajectoryWaypointMsg _PhysicsStateMsg;
    typedef __QUICKDEV_FUNCTION_TYPE<void( _TrajectoryWaypointMsg::ConstPtr const & )> _PhysicsStateCallback;

protected:
    _PhysicsStateMsg::ConstPtr physics_state_msg_;

private:
    ros::MultiSubscriber<> multi_sub_;

    QUICKDEV_MAKE_POLICY_FUNCS( SeabeeMovement )

    QUICKDEV_DECLARE_POLICY_CONSTRUCTOR( SeabeeMovement )
    {
        //
    }

    QUICKDEV_ENABLE_INIT()
    {
        auto & nh_rel = _MakeTrajectoryActionClientPolicy::getNodeHandle();

        multi_sub_.addSubscriber( nh_rel, "physics_state", &SeabeeMovementPolicy::physicsStateCB, this );

//        _MakeTrajectoryActionClientPolicy::registerDoneCB( quickdev::auto_bind( &SeabeeMovementPolicy::makeTrajectoryActionDoneCB, this ) );
//        _FollowTrajectoryActionClientPolicy::registerDoneCB( quickdev::auto_bind( &SeabeeMovementPolicy::followTrajectoryActionDoneCB, this ) );

        initPolicies<_MakeTrajectoryActionClientPolicy>( "action_name", std::string( "make_trajectory" ) );
        initPolicies<_FollowTrajectoryActionClientPolicy>( "action_name", std::string( "follow_trajectory" ) );

        initPolicies<quickdev::policy::ALL>();

        QUICKDEV_SET_INITIALIZED();
    }

/*
    QUICKDEV_DECLARE_ACTION_DONE_CALLBACK( makeTrajectoryActionDoneCB, _MakeTrajectoryAction )
    {
        //
    }

    QUICKDEV_DECLARE_ACTION_DONE_CALLBACK( followTrajectoryActionDoneCB, _FollowTrajectoryAction )
    {
        //
    }
*/

    QUICKDEV_DECLARE_MESSAGE_CALLBACK( physicsStateCB, _PhysicsStateMsg )
    {
        physics_state_msg_ = msg;
    }

    // #########################################################################################################################################
    //! Return the Pose of the landmark with the given name
    Pose getPose( std::string const & to_frame, std::string const & from_frame = "seabee" )
    {
        auto transform = _TfTranceiverPolicy::tryLookupTransform( from_frame, to_frame );
        return unit::convert<Pose>( btTransform( transform ) );
    }

    //! Return the Pose of the sub
    /*!
     * - Calls getPose( "seabee" )
     */
    Pose getCurrentPose()
    {
        return getPose( "world", "seabee" );
    }

    // #########################################################################################################################################
    //! Fire the given device
    quickdev::ActionToken<boost::thread> fireDevice( FiringDevice const & device );

    // #########################################################################################################################################
    //! Fire the given shooter
    /*!
     * - Calls fireDevice( Shooter( shooter_id ) )
     */
    quickdev::ActionToken<boost::thread> fireShooter( Shooter::Id const & shooter_id );

    //! Fire the given dropper
    /*!
     * - Calls fireDevice( Dropper( dropper_id ) )
     */
    quickdev::ActionToken<boost::thread> fireDropper( Dropper::Id const & dropper_id );

    // #########################################################################################################################################
    //! Move to the given pose
    /*!
     * - moveTo( getCurrentPose() + Orientation( 90 ) ) will rotate 90 degrees CCW (see faceTo( Orientation ) )
     * - moveTo( getCurrentPose() + Position( 0, 0, -1 ) will dive one meter
     */
    _FollowTrajectoryActionClientPolicy::_ActionToken moveTo( Pose const & pose )
    {
        if( !physics_state_msg_ ) return _FollowTrajectoryActionClientPolicy::_ActionToken();

        _MakeTrajectoryActionClientPolicy::_GoalMsg make_trajectory_goal;

        make_trajectory_goal.waypoints.push_back( *physics_state_msg_ );

        _TrajectoryWaypointMsg ending_waypoint;
        ending_waypoint.pose.pose = unit::make_unit( pose );
        // velocity will default to zero
        make_trajectory_goal.waypoints.push_back( ending_waypoint );

        auto make_trajectory_token = _MakeTrajectoryActionClientPolicy::sendGoal( make_trajectory_goal );
        auto make_trajectory_result = make_trajectory_token.get( 1.0 );

        if( make_trajectory_result->trajectory.intervals.size() > 0 )
        {
            _FollowTrajectoryActionClientPolicy::_GoalMsg follow_trajectory_goal;
            follow_trajectory_goal.trajectory = make_trajectory_result->trajectory;

            return _FollowTrajectoryActionClientPolicy::sendGoal( follow_trajectory_goal );
        }
        return _FollowTrajectoryActionClientPolicy::_ActionToken();
    }

    //! Move to some relative position
    /*!
     * - Same as moveTo( getCurrentPose() + position )
     */
    _FollowTrajectoryActionClientPolicy::_ActionToken moveTo( Position const & position )
    {
        return moveTo( getCurrentPose() + position );
    }

    //! Move to some relative orientation
    /*!
     * - Same as moveTo( getCurrentPose() + orientation )
     */
    _FollowTrajectoryActionClientPolicy::_ActionToken moveTo( Orientation const & orientation )
    {
        return moveTo( getCurrentPose() + orientation );
    }

    // #########################################################################################################################################
    //! Face the given position
    _FollowTrajectoryActionClientPolicy::_ActionToken faceTo( Position const & position );

    //! Face at the given orientation
    _FollowTrajectoryActionClientPolicy::_ActionToken faceTo( Orientation const & orientation );

    // #########################################################################################################################################
    //! Strafe around pose.position at current distance until our orientation matches pose.orientation + Degrees( 180 )
    _FollowTrajectoryActionClientPolicy::_ActionToken strafeAround( Pose const & pose );

    //! Strafe around pose.position at current distance until our orientation matches orientation
    _FollowTrajectoryActionClientPolicy::_ActionToken strafeAround( Pose const & pose, Orientation const & orientation );

    //! Strafe around pose.position at distance until ... (see above)
    _FollowTrajectoryActionClientPolicy::_ActionToken strafeAround( Pose const & pose, double distance, ... );

    /* This is the client's main means of accessing and updating a policy; un-comment and change args as appropriate

    void update( args... )
    {
        // When using init(), it's useful to be able to detect whether our init() function as been called
        // One option is to simply read the member "initialized_"
        // Alternatively, there are some simple macros for notifying/responding to situations where init() has not been called
        // These macros can be used anywhere within our policy as long as our init() function has been declared with QUICKDEV_ENABLE_INIT{}
        //
        // If we're not initialized yet, warn and continue
        // QUICKDEV_CHECK_INITIALIZED();
        //
        // If we're not initialized yet, warn and return immediately
        // QUICKDEV_ASSERT_INITIALIZED();
        //
    }*/
};

#endif // SEABEE3COMMON_SEABEEMOVEMENTPOLICY_H_
