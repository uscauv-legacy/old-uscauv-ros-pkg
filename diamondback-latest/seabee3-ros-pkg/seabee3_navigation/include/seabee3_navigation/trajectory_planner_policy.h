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

    void registerPlanTrajectoryCB( _MakeTrajectoryActionServerPolicy::_ExecuteCallback const & callback )
    {
        _MakeTrajectoryActionServerPolicy::registerExecuteCB( callback );
    }

    QUICKDEV_DECLARE_ACTION_PREEMPT_CALLBACK( makeTrajectoryActionPreemptCB, _MakeTrajectoryAction )
    {
        //
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
