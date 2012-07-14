/***************************************************************************
 *  include/contour_matcher/contour_matcher_node.h
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

#ifndef CONTOURMATCHER_CONTOURMATCHERNODE_H_
#define CONTOURMATCHER_CONTOURMATCHERNODE_H_

#include <quickdev/node.h>

// policies
#include <quickdev/action_server_policy.h>

// objects
#include <quickdev/multi_subscriber.h>

// utils
#include <contour_matcher/contour.h>

// actions
#include <seabee3_actions/MatchContoursAction.h>

// msgs
#include <seabee3_msgs/ContourArray.h>

typedef seabee3_msgs::ContourArray _ContourArrayMsg;
typedef seabee3_msgs::Contour _ContourMsg;

typedef seabee3_actions::MatchContoursAction _MatchContoursAction;

typedef quickdev::ActionServerPolicy<_MatchContoursAction> _MatchContoursActionServerPolicy;

// subscribe to contours from contour_finder
// when request comes in, ignore new contours until request is completed

QUICKDEV_DECLARE_NODE( ContourMatcher, _MatchContoursActionServerPolicy )

QUICKDEV_DECLARE_NODE_CLASS( ContourMatcher )
{
protected:
    ros::MultiSubscriber<> multi_sub_;

    QUICKDEV_DECLARE_NODE_CONSTRUCTOR( ContourMatcher )
    {
        //
    }
    QUICKDEV_SPIN_FIRST()
    {
        QUICKDEV_GET_RUNABLE_NODEHANDLE( nh_rel );

        multi_sub_.addSubscriber( nh_rel, "input_contours", &ContourMatcherNode::contoursCB, this );

        initPolicies<quickdev::policy::ALL>();
    }

    QUICKDEV_DECLARE_ACTION_EXECUTE_CALLBACK( matchContoursActionExecuteCB, _MatchContoursAction )
    {
        //
    }

    QUICKDEV_DECLARE_MESSAGE_CALLBACK( contoursCB, _ContourArrayMsg )
    {
        //
    }

    QUICKDEV_SPIN_ONCE()
    {
        //
    }
};

#endif // CONTOURMATCHER_CONTOURMATCHERNODE_H_
