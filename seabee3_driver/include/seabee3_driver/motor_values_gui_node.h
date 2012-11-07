/***************************************************************************
 *  include/seabee3_driver/motor_values_gui_node.h
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

#ifndef SEABEE3DRIVER_MOTORVALUESGUINODE_H_
#define SEABEE3DRIVER_MOTORVALUESGUINODE_H_

#include <quickdev/node.h>

// utils
#include <seabee3_common/movement.h>

// policies
#include <quickdev/reconfigure_policy.h>

// objects
#include <quickdev/multi_publisher.h>

// msgs
#include <seabee3_msgs/MotorVals.h>

// cfgs
#include <seabee3_driver/MotorValuesConfig.h>

typedef seabee3_msgs::MotorVals _MotorValsMsg;

typedef seabee3_driver::MotorValuesConfig _MotorValuesCfg;

typedef quickdev::ReconfigurePolicy<_MotorValuesCfg> _MotorValuesLiveParams;

using namespace seabee3_common::movement;

QUICKDEV_DECLARE_NODE( MotorValuesGUI, _MotorValuesLiveParams )

QUICKDEV_DECLARE_NODE_CLASS( MotorValuesGUI )
{
    ros::MultiPublisher<> multi_pub_;

    _MotorValsMsg motor_vals_msg_;

    QUICKDEV_DECLARE_NODE_CONSTRUCTOR( MotorValuesGUI )
    {
        auto & mask = motor_vals_msg_.mask;
        for( auto mask_it = mask.begin(); mask_it != mask.end(); ++mask_it )
        {
            *mask_it = 1;
        }
    }

    QUICKDEV_SPIN_FIRST()
    {
        _MotorValuesLiveParams::registerCallback( quickdev::auto_bind( &MotorValuesGUINode::reconfigureCB, this ) );
        initPolicies<quickdev::policy::ALL>();


        QUICKDEV_GET_RUNABLE_NODEHANDLE( nh_rel );
        multi_pub_.addPublishers<_MotorValsMsg>( nh_rel, { "motor_vals" } );
    }

    QUICKDEV_SPIN_ONCE()
    {
        multi_pub_.publish( "motor_vals", motor_vals_msg_ );
    }

    QUICKDEV_DECLARE_RECONFIGURE_CALLBACK( reconfigureCB, _MotorValuesCfg )
    {
        if( config.apply && !config_.apply )
        {
            motor_vals_msg_.motors[MotorControllerIDs::FWD_RIGHT_THRUSTER] = config.fwd_right_thruster;
            motor_vals_msg_.motors[MotorControllerIDs::FWD_LEFT_THRUSTER] =  config.fwd_left_thruster;
            motor_vals_msg_.motors[MotorControllerIDs::DEPTH_FRONT_THRUSTER] = config.depth_front_thruster;
            motor_vals_msg_.motors[MotorControllerIDs::DEPTH_BACK_THRUSTER] =  config.depth_back_thruster;
            motor_vals_msg_.motors[MotorControllerIDs::STRAFE_TOP_THRUSTER] =    config.strafe_top_thruster;
            motor_vals_msg_.motors[MotorControllerIDs::STRAFE_BOTTOM_THRUSTER] = config.strafe_bottom_thruster;
            motor_vals_msg_.motors[6] = config.slot_7;
            motor_vals_msg_.motors[7] = config.slot_8;
            motor_vals_msg_.motors[8] = config.slot_9;
        }
    }
};

#endif // SEABEE3DRIVER_MOTORVALUEGUINODE_H_
