/***************************************************************************
 *  include/seabee3_navigation/trajectory_follower_node.h
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

#ifndef SEABEE3NAVIGATION_TRAJECTORYFOLLOWERNODE_H_
#define SEABEE3NAVIGATION_TRAJECTORYFOLLOWERNODE_H_

#include <quickdev/node.h>

// policies
#include <quickdev/action_server_policy.h>

// actions
#include <seabee3_actions/FollowTrajectoryAction.h>

typedef seabee3_actions::FollowTrajectoryAction _FollowTrajectoryAction;

typedef quickdev::ActionServerPolicy<_FollowTrajectoryAction> _FollowTrajectoryActionServerPolicy;

QUICKDEV_DECLARE_NODE( TrajectoryFollower, _FollowTrajectoryActionServerPolicy )

QUICKDEV_DECLARE_NODE_CLASS( TrajectoryFollower )
{
    QUICKDEV_DECLARE_NODE_CONSTRUCTOR( TrajectoryFollower )
    {
        initPolicies<QUICKDEV_GET_RUNABLE_POLICY()>();
    }

    QUICKDEV_SPIN_FIRST()
    {
        _FollowTrajectoryActionServerPolicy::registerExecuteCB( quickdev::auto_bind( &TrajectoryFollowerNode::followTrajectoryActionExecuteCB, this ) );
        _FollowTrajectoryActionServerPolicy::registerPreemptCB( quickdev::auto_bind( &TrajectoryFollowerNode::followTrajectoryActionPreemptCB, this ) );

        initPolicies<_FollowTrajectoryActionServerPolicy>( "action_name_param", std::string( "follow_trajectory" ) );

        initPolicies<quickdev::policy::ALL>();
    }

    QUICKDEV_SPIN_ONCE()
    {
        //
    }

    QUICKDEV_DECLARE_ACTION_EXECUTE_CALLBACK( followTrajectoryActionExecuteCB, _FollowTrajectoryAction )
    {
        //
    }

    QUICKDEV_DECLARE_ACTION_PREEMPT_CALLBACK( followTrajectoryActionPreemptCB, _FollowTrajectoryAction )
    {
        //
    }
};

#endif // SEABEE3NAVIGATION_TRAJECTORYFOLLOWERNODE_H_
