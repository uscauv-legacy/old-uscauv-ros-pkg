/***************************************************************************
 *  include/seabee3_trajectory_planner/trajectory_planner_policy.h
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

#ifndef SEABEE3TRAJECTORYPLANNER_TRAJECTORYPLANNERPOLICY_H_
#define SEABEE3TRAJECTORYPLANNER_TRAJECTORYPLANNERPOLICY_H_

#include <quickdev/policy.h>

// Declare private storage for our types, etc
// This namespace can be retrieved later on using QUICKDEV_GET_POLICY_NS( TrajectoryPlanner )
//
QUICKDEV_DECLARE_POLICY_NS( TrajectoryPlanner )
{
    // Typedefs for parent policies should be declared here
    // Say we had:
    //
    // typedef SomePolicy<SomeType> _SomePolicy;
    //
    // When we declare our policy, we can simply specify _SomePolicy as one of the policies we want to use
    // When we want to get _SomePolicy anywhere else in the code:
    //
    // QUICKDEV_GET_POLICY_NS( TrajectoryPlanner )::_SomePolicy
    //
    // In the event that we need to make lots of calls to _SomePolicy, we can store a reference to it using:
    //
    // auto & some_policy = QUICKDEV_GET_POLICY_NS( TrajectoryPlanner )::_SomePolicy::getInstance();
    // some_policy.someFunction();
    // some_policy.someFunction2();
    //
    // Otherwise we'd have to do:
    //
    // QUICKDEV_GET_POLICY_NS( TrajectoryPlanner )::_SomePolicy::someFunction();
    // QUICKDEV_GET_POLICY_NS( TrajectoryPlanner )::_SomePolicy::someFunction2();
    //
    typedef quickdev::Policy _Policy;
}

// Declare a policy called TrajectoryPlannerPolicy; it will inherit the functionalify of all the policies that follow in the list of arguments
// For example, to make a policy called SomePolicy that uses Policy1 and Policy2:
//
// QUICKDEV_DECLARE_POLICY( Some, Policy1, Policy2 )
//
// "Policy" is automatically appended to the first argument
//
QUICKDEV_DECLARE_POLICY( TrajectoryPlanner, _Policy )

// Declare a class called TrajectoryPlannerPolicy
//
QUICKDEV_DECLARE_POLICY_CLASS( TrajectoryPlanner )
{
    // Create utility functions for this policy
    //
    QUICKDEV_MAKE_POLICY_FUNCS( TrajectoryPlanner )

    // Variable initializations can be appended to this constructor as a comma-separated list:
    //
    // QUICKDEV_DECLARE_POLICY_CONSTRUCTOR( TrajectoryPlanner ), member1_( some_value ), member2_( some_other_value ){}
    //
    // Most initialization will need to be done in init() ( QUICKDEV_ENABLE_INIT ) since a variadic template can't easily be passed through this
    // constructor ( yet-unimplemented feature of g++ )
    //
    // Note that if QUICKDEV_ENABLE_INIT is used, a member, "initialized_", should be set to false during construction
    //
    QUICKDEV_DECLARE_POLICY_CONSTRUCTOR( TrajectoryPlanner )
    {
        //
    }

    /* Un-comment to enable initialization of this policy; this is the client's main means of passing compile-time args to a policy

    QUICKDEV_ENABLE_INIT()
    {
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
    }*/

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

#endif // SEABEE3TRAJECTORYPLANNER_TRAJECTORYPLANNERPOLICY_H_
