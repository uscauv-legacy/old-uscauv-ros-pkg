/***************************************************************************
 *  include/seabee3_navigation/trajectory_planner_policy.h
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

#ifndef SEABEE3NAVIGATION_TRAJECTORYPLANNERPOLICY_H_
#define SEABEE3NAVIGATION_TRAJECTORYPLANNERPOLICY_H_

// policies
#include <quickdev/action_server_policy.h>

// utils
#include <seabee3_navigation/trajectory_message_conversions.h>
#include <quickdev/math.h>

// actions
#include <seabee3_actions/MakeTrajectoryAction.h>

QUICKDEV_DECLARE_POLICY_NS( TrajectoryPlanner )
{
    typedef seabee3_actions::MakeTrajectoryAction _MakeTrajectoryAction;

    typedef quickdev::ActionServerPolicy<_MakeTrajectoryAction> _MakeTrajectoryActionServerPolicy;
}

QUICKDEV_DECLARE_POLICY( TrajectoryPlanner, _MakeTrajectoryActionServerPolicy )

QUICKDEV_DECLARE_POLICY_CLASS( TrajectoryPlanner )
{
public:
    typedef QUICKDEV_GET_POLICY_NS( TrajectoryPlanner )::_MakeTrajectoryAction _MakeTrajectoryAction;
    typedef QUICKDEV_GET_POLICY_NS( TrajectoryPlanner )::_MakeTrajectoryActionServerPolicy _MakeTrajectoryActionServerPolicy;

    typedef seabee3_msgs::Trajectory _TrajectoryMsg;
    typedef seabee3_msgs::TrajectoryInterval _TrajectoryIntervalMsg;
    typedef seabee3_msgs::TrajectoryWaypoint _TrajectoryWaypointMsg;

protected:
    btVector3 max_linear_velocity_;
    btVector3 max_angular_velocity_;
    btTransform max_velocity_;
    btVector3 max_linear_acceleration_;
    btVector3 max_angular_acceleration_;
    btTransform max_acceleration_;
    double time_resolution_;

    btTransform current_pose_;
    btVector3 current_linear_velocity_;
    btVector3 current_angular_velocity_;
    _TrajectoryWaypointMsg last_output_waypoint_;

    QUICKDEV_MAKE_POLICY_FUNCS( TrajectoryPlanner )

    QUICKDEV_DECLARE_POLICY_CONSTRUCTOR( TrajectoryPlanner )
    {
        //
    }

    QUICKDEV_ENABLE_INIT()
    {
        QUICKDEV_GET_NODEHANDLE( nh_rel );

        _MakeTrajectoryActionServerPolicy::registerPreemptCB( quickdev::auto_bind( &TrajectoryPlannerPolicy::makeTrajectoryActionPreemptCB, this ) );

        auto action_name = quickdev::policy::readPolicyParam( nh_rel, "action_name_param", "action_name", std::string( "make_trajectory" ), args... );

        auto params = quickdev::ParamReader::readParam<XmlRpc::XmlRpcValue>( nh_rel, "params" );

        auto constraints = quickdev::ParamReader::getXmlRpcValue<XmlRpc::XmlRpcValue>( params, "constraints" );

        time_resolution_ = quickdev::ParamReader::getXmlRpcValue<double>( constraints, "time_resolution" );

        auto velocity = quickdev::ParamReader::getXmlRpcValue<XmlRpc::XmlRpcValue>( constraints, "velocity" );

        auto linear_velocity = quickdev::ParamReader::getXmlRpcValue<XmlRpc::XmlRpcValue>( velocity, "linear" );
        max_linear_velocity_.setX( quickdev::ParamReader::getXmlRpcValue<double>( linear_velocity, "x", 0.3 ) );
        max_linear_velocity_.setY( quickdev::ParamReader::getXmlRpcValue<double>( linear_velocity, "y", 0.2 ) );
        max_linear_velocity_.setZ( quickdev::ParamReader::getXmlRpcValue<double>( linear_velocity, "z", 0.05 ) );

        auto angular_velocity = quickdev::ParamReader::getXmlRpcValue<XmlRpc::XmlRpcValue>( velocity, "angular" );
        max_angular_velocity_.setX( quickdev::ParamReader::getXmlRpcValue<double>( angular_velocity, "x", 0.9 ) );
        max_angular_velocity_.setY( quickdev::ParamReader::getXmlRpcValue<double>( angular_velocity, "y", 0.0 ) );
        max_angular_velocity_.setZ( quickdev::ParamReader::getXmlRpcValue<double>( angular_velocity, "z", 0.0 ) );

        max_velocity_ = btTransform( unit::convert<btQuaternion>( max_angular_velocity_ ), max_linear_velocity_ );

        auto acceleration = quickdev::ParamReader::getXmlRpcValue<XmlRpc::XmlRpcValue>( constraints, "acceleration" );

        auto linear_acceleration = quickdev::ParamReader::getXmlRpcValue<XmlRpc::XmlRpcValue>( acceleration, "linear" );
        max_linear_acceleration_.setX( quickdev::ParamReader::getXmlRpcValue<double>( linear_acceleration, "x", 0.3 ) );
        max_linear_acceleration_.setY( quickdev::ParamReader::getXmlRpcValue<double>( linear_acceleration, "y", 0.2 ) );
        max_linear_acceleration_.setZ( quickdev::ParamReader::getXmlRpcValue<double>( linear_acceleration, "z", 0.05 ) );

        auto angular_acceleration = quickdev::ParamReader::getXmlRpcValue<XmlRpc::XmlRpcValue>( acceleration, "angular" );
        max_angular_acceleration_.setX( quickdev::ParamReader::getXmlRpcValue<double>( angular_acceleration, "x", 0.9 ) );
        max_angular_acceleration_.setY( quickdev::ParamReader::getXmlRpcValue<double>( angular_acceleration, "y", 0.0 ) );
        max_angular_acceleration_.setZ( quickdev::ParamReader::getXmlRpcValue<double>( angular_acceleration, "z", 0.0 ) );

        max_acceleration_ = btTransform( unit::convert<btQuaternion>( max_angular_acceleration_ ), max_linear_acceleration_ );

        initPolicies<_MakeTrajectoryActionServerPolicy>( "action_name_param", action_name );

        // A variadic template called "args" is available in this function
        // Use getFirstOfType<Type>( args... ) to parse unnamed args
        // Use getMetaParam<Type>( "param_name", args... ) or getMetaParamDef<Type>( "param_name", default, args... ) to parse named args
        //
        // Say we want to get a parameter of type "__SomeType" with the key "some_value_param"; if the key is not found, "default_value"
        // will be returned
        // Note that this will fail at compile time if "__SomeType" does not exist in the list of arguments passed to this function:
        //
        // const auto some_value_param = quickdev::getMetaParamDef<__SomeType>( "some_value_param", default_value, args... );
        //
        // After we've read all our parameters, we should be sure to set our policy's initialized state to true so that we can respond
        // accordingly if the user fails to call init() for our policy
        //
        QUICKDEV_SET_INITIALIZED();
    }

    template<class... __Args>
    void registerPlanTrajectoryCB( __Args... args )
    {
        _MakeTrajectoryActionServerPolicy::registerExecuteCB( args... );
    }

    QUICKDEV_DECLARE_ACTION_PREEMPT_CALLBACK( makeTrajectoryActionPreemptCB, _MakeTrajectoryAction )
    {
        //
    }

    // update the simulated velocity, respecting any user-specified dynamics constraints
    void changeVelocity( btTransform const & desired_velocity )
    {
        auto const desired_linear_velocity = desired_velocity.getOrigin();
        auto const desired_angular_velocity = unit::convert<btVector3>( desired_velocity.getRotation() );

        if( fabs( current_linear_velocity_.getX() ) > max_linear_velocity_.getX() ) current_linear_velocity_.setX( quickdev::sign( current_linear_velocity_.getX() ) * max_linear_velocity_.getX() );
        if( fabs( current_linear_velocity_.getY() ) > max_linear_velocity_.getY() ) current_linear_velocity_.setY( quickdev::sign( current_linear_velocity_.getY() ) * max_linear_velocity_.getY() );
        if( fabs( current_linear_velocity_.getZ() ) > max_linear_velocity_.getZ() ) current_linear_velocity_.setZ( quickdev::sign( current_linear_velocity_.getZ() ) * max_linear_velocity_.getZ() );

        if( fabs( current_angular_velocity_.getX() ) > max_angular_velocity_.getX() ) current_angular_velocity_.setX( quickdev::sign( current_angular_velocity_.getX() ) * max_angular_velocity_.getX() );
        if( fabs( current_angular_velocity_.getY() ) > max_angular_velocity_.getY() ) current_angular_velocity_.setY( quickdev::sign( current_angular_velocity_.getY() ) * max_angular_velocity_.getY() );
        if( fabs( current_angular_velocity_.getZ() ) > max_angular_velocity_.getZ() ) current_angular_velocity_.setZ( quickdev::sign( current_angular_velocity_.getZ() ) * max_angular_velocity_.getZ() );
    }

    // add the given interval to the list of intervals
    void addInterval( _TrajectoryIntervalMsg const & interval, std::vector<_TrajectoryIntervalMsg> & intervals )
    {
        intervals.push_back( interval );
    }

    // create an interval from the given components, then add it to the list of intervals
    void addInterval( double const & duration, btTransform const & acceleration, btTransform const & initial_pose, btTransform const & initial_velocity, std::vector<_TrajectoryIntervalMsg> & intervals )
    {
        _TrajectoryIntervalMsg interval;
//        interval.length = duration;
        interval.initial_state.pose.pose = unit::implicit_convert( initial_pose );
        interval.initial_state.velocity.twist = unit::implicit_convert( initial_velocity );
        interval.acceleration = acceleration;
        addInterval( interval, intervals );
    }

    // create an interval from the given components (assuming that we started at the most recent waypoint), then add it to the list of intervals
    // and update our copy of the most recent waypoint
    void addInterval( double const & duration, btTransform const & acceleration, std::vector<_TrajectoryIntervalMsg> & intervals )
    {
        _TrajectoryIntervalMsg interval;
        interval.duration = duration;
        interval.initial_state = last_output_waypoint_;
        interval.acceleration = acceleration;

        addInterval( interval, intervals );

        last_output_waypoint_.pose.pose = unit::implicit_convert( current_pose_ );
        last_output_waypoint_.velocity.twist.linear = unit::implicit_convert( current_linear_velocity_ );
        last_output_waypoint_.velocity.twist.angular = unit::implicit_convert( current_angular_velocity_ );
    }

    // accelerate to a given linear/angular velocity respecting any user-specified constraints by generating and adding intervals to the list of intervals
    void accelerateTo( btTransform const & desired_velocity, std::vector<_TrajectoryIntervalMsg> & intervals )
    {
        auto const desired_linear_velocity = desired_velocity.getOrigin();
        auto const desired_angular_velocity = unit::convert<btVector3>( desired_velocity.getRotation() );

        while
        (
            current_linear_velocity_.getX() < desired_linear_velocity.getX() ||
            current_linear_velocity_.getY() < desired_linear_velocity.getY() ||
            current_linear_velocity_.getZ() < desired_linear_velocity.getZ() ||
            current_angular_velocity_.getX() < desired_angular_velocity.getX() ||
            current_angular_velocity_.getY() < desired_angular_velocity.getY() ||
            current_angular_velocity_.getZ() < desired_angular_velocity.getZ()
        )
        {
            // we change velocity on a per-axis basis at the maximum acceleration allowed as long as we're not already moving at the desired velocity along each axis
            // v = at + v0
            auto velocity = max_acceleration_ * time_resolution_ + current_velocity_;
            auto linear_velocity = velocity.getOrigin();
            auto angular_velocity = unit::convert<btVector3>( velocity.getRotation() );

            if( current_linear_velocity_.getX() >= desired_linear_velocity.getX() ) linear_velocity.setX( 0 );
            if( current_linear_velocity_.getY() >= desired_linear_velocity.getY() ) linear_velocity.setY( 0 );
            if( current_linear_velocity_.getZ() >= desired_linear_velocity.getZ() ) linear_velocity.setZ( 0 );

            if( current_angular_velocity_.getX() >= desired_angular_velocity.getX() ) angular_velocity.setX( 0 );
            if( current_angular_velocity_.getY() >= desired_angular_velocity.getY() ) angular_velocity.setY( 0 );
            if( current_angular_velocity_.getZ() >= desired_angular_velocity.getZ() ) angular_velocity.setZ( 0 );

            // s = at^2/2 + v0t + s0
            current_pose_ = max_acceleration_ * time_resolution_ * time_resolution_ / 2 + current_velocity_ * time_resolution_ + current_pose_;

            // note that we update the velocity after updating the pose
            changeVelocity( btTransform( unit::convert<btQuaternion>( angular_velocity ), linear_velocity ) );

            addInterval( time_resolution,  );
        }
    }

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

#endif // SEABEE3NAVIGATION_TRAJECTORYPLANNERPOLICY_H_
