/***************************************************************************
 *  include/landmark_finder/landmark_finder_node.h
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

#ifndef LANDMARKFINDER_LANDMARKFINDERNODE_H_
#define LANDMARKFINDER_LANDMARKFINDERNODE_H_

#include <quickdev/node.h>

// policies
#include <quickdev/action_client_policy.h>

// objects
#include <quickdev/multi_subscriber.h>

// actions
#include <seabee3_actions/ConfigureAction.h>

// msgs
#include <seabee3_msgs/NamedImageArray.h>

typedef seabee3_actions::ConfigureAction _ConfigureAction;

typedef quickdev::ActionClientPolicy<_ConfigureAction> _ConfigureActionClientPolicy;

// responsible for setting landmark-related filters
// for example, if pipes are requested

QUICKDEV_DECLARE_NODE( LandmarkFinder, _ConfigureActionClientPolicy )

QUICKDEV_DECLARE_NODE_CLASS( LandmarkFinder )
{
    ros::MultiSubscriber<> multi_sub_;

    QUICKDEV_DECLARE_NODE_CONSTRUCTOR( LandmarkFinder )
    {
        //
    }

    QUICKDEV_SPIN_FIRST()
    {
        QUICKDEV_GET_RUNABLE_NODEHANDLE( nh_rel );

        // when our goal is done, call this callback
        _ConfigureActionClientPolicy::registerDoneCB( quickdev::auto_bind( &LandmarkFinderNode::configureActionDoneCB, this ) );

        initPolicies<_ConfigureActionClientPolicy>( "action_name_param", std::string( "set_color_filter" ) );

        initPolicies<quickdev::policy::ALL>();
    }

    QUICKDEV_DECLARE_ACTION_DONE_CALLBACK( configureActionDoneCB, _ConfigureAction )
    {
        //
    }

    QUICKDEV_SPIN_ONCE()
    {
        //
    }
};

#endif // LANDMARKFINDER_LANDMARKFINDERNODE_H_
