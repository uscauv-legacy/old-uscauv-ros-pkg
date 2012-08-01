/***************************************************************************
 *  include/seabee3_navigation/linear_trajectory_planner_node.h
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

#ifndef SEABEE3NAVIGATION_LINEARTRAJECTORYPLANNERNODE_H_
#define SEABEE3NAVIGATION_LINEARTRAJECTORYPLANNERNODE_H_

#include <quickdev/node.h>

// policies
#include <seabee3_navigation/trajectory_planner_policy.h>

// utils
#include <quickdev/math.h>

typedef TrajectoryPlannerPolicy _TrajectoryPlannerPolicy;

QUICKDEV_DECLARE_NODE( LinearTrajectoryPlanner, _TrajectoryPlannerPolicy )

QUICKDEV_DECLARE_NODE_CLASS( LinearTrajectoryPlanner )
{
    QUICKDEV_DECLARE_NODE_CONSTRUCTOR( LinearTrajectoryPlanner )
    {
        initPolicies<QUICKDEV_GET_RUNABLE_POLICY()>();
    }

    QUICKDEV_SPIN_FIRST()
    {
        _TrajectoryPlannerPolicy::registerPlanTrajectoryCB( quickdev::auto_bind( &LinearTrajectoryPlannerNode::planTrajectory, this ) );
        initPolicies<quickdev::policy::ALL>();
    }

    QUICKDEV_SPIN_ONCE()
    {
        //
    }

    QUICKDEV_DECLARE_ACTION_EXECUTE_CALLBACK( planTrajectory, _TrajectoryPlannerPolicy::_MakeTrajectoryAction )
    {
        // if there are not at least two waypoints, abort the action; we need to at least know where we are and where we want to go
        if( goal->waypoints.size() <= 2 ) return _TrajectoryPlannerPolicy::setAborted();

        // get a ref to the waypoints
        auto const & waypoints = goal->waypoints;

        // allocate a new result message
        _TrajectoryPlannerPolicy::_MakeTrajectoryActionServerPolicy::_ResultMsg result;

        // get a reference to the vector of intervals in the new trajectory
        auto & intervals = result.trajectory.intervals;

        // get an iterator to the first waypoint
        auto waypoints_it = waypoints.cbegin();

        _TrajectoryPlannerPolicy::current_pose_ = unit::make_unit( waypoints_it->pose );
        _TrajectoryPlannerPolicy::current_velocity_ = unit::make_unit( waypoints_it->velocity );

        waypoints_it ++;

        // for each waypoint
        for( ; waypoints_it != waypoints.cend(); ++waypoints_it )
        {
            auto const & waypoint = *waypoints_it;

            // slow down
            _TrajectoryPlannerPolicy::accelerateTo( btTransform( btQuaternion( 0, 0, 0 ), btVector3( 0, 0, 0 ) ), intervals );

            auto const waypoint_tf = unit::convert<btTransform>( waypoint.pose );

            // turn to face the current waypoint
            alignTo( waypoint_tf, intervals );

            // drive up to the waypoint
            moveTo( waypoint_tf, intervals );

            // if this is the last waypoint, turn to face the waypoint's orientation
            if( waypoints_it == waypoints.cend() - 1 ) faceTo( waypoint_tf, intervals );
        }

        // return the new trajectory
        return _TrajectoryPlannerPolicy::setSuccessful( result );
    }
};

#endif // SEABEE3NAVIGATION_LINEARTRAJECTORYPLANNERNODE_H_
