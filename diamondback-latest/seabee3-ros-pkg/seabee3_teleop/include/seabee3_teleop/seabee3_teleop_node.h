/***************************************************************************
 *  include/seabee3_teleop/seabee3_teleop_node.h
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

#ifndef SEABEE3TELEOP_SEABEE3TELEOPNODE_H_
#define SEABEE3TELEOP_SEABEE3TELEOPNODE_H_

#include <quickdev/node.h>
#include <quickdev/joystick_policy.h>
#include <quickdev/service_client_policy.h>
#include <seabee3_driver/FiringDeviceAction.h>

typedef quickdev::JoystickPolicy _JoystickPolicy;
typedef seabee3_driver::FiringDeviceAction _FiringDeviceActionService;
typedef quickdev::ServiceClientPolicy<_FiringDeviceActionService, 0> _Shooter1ServiceClient;
typedef quickdev::ServiceClientPolicy<_FiringDeviceActionService, 1> _Shooter2ServiceClient;
typedef quickdev::ServiceClientPolicy<_FiringDeviceActionService, 2> _Dropper1ServiceClient;
typedef quickdev::ServiceClientPolicy<_FiringDeviceActionService, 3> _Dropper2ServiceClient;

QUICKDEV_DECLARE_NODE( Seabee3Teleop, _JoystickPolicy, _Shooter1ServiceClient, _Shooter2ServiceClient, _Dropper1ServiceClient, _Dropper2ServiceClient )

QUICKDEV_DECLARE_NODE_CLASS( Seabee3Teleop )
{
public:
    int current_firing_device_;
    int num_firing_devices_;
    _JoystickPolicy::_Axis::_Name current_button_;

    QUICKDEV_DECLARE_NODE_CONSTRUCTOR( Seabee3Teleop ),
        current_firing_device_( 0 ),
        num_firing_devices_( 4 ),
        current_button_( "" )
    {
        //
    }

    bool tryGetButtonLock( const _JoystickPolicy::_Axis & axis, const _JoystickPolicy::_JoystickMsg::ConstPtr & msg )
    {
        if( axis.getValueAsButton( msg ) > 0 )
        {
            if( current_button_ == "" )
            {
                //PRINT_INFO( "Axis [ %s ] gained lock", axis.name_.c_str() );

                current_button_ = axis.name_;
                return true;
            }
        }
        else
        {
            if( current_button_ == axis.name_ )
            {
                //PRINT_INFO( "Axis [ %s ] released lock", axis.name_.c_str() );
                current_button_ = "";
            }
        }

        return false;
    }

    QUICKDEV_DECLARE_MESSAGE_CALLBACK( joystickCB, _JoystickPolicy::_JoystickMsg )
    {
        if( JoystickPolicy::isEnabled() )
        {
            const auto & next_firing_device_axis = _JoystickPolicy::getAxis( "next_firing_device" );
            const auto & prev_firing_device_axis = _JoystickPolicy::getAxis( "prev_firing_device" );
            const auto & fire_device_axis = _JoystickPolicy::getAxis( "fire_device" );

            if( tryGetButtonLock( next_firing_device_axis, msg ) ) ++current_firing_device_;
            if( tryGetButtonLock( prev_firing_device_axis, msg ) ) --current_firing_device_;

            if( num_firing_devices_ > 0 )
            {
                if( current_firing_device_ > num_firing_devices_ - 1 ) current_firing_device_ = 0;
                if( current_firing_device_ < 0 ) current_firing_device_ = num_firing_devices_ - 1;
            }

            if( tryGetButtonLock( fire_device_axis, msg ) ) fireCurrentDevice();
        }
    }

    void fireCurrentDevice()
    {
        PRINT_INFO( "Firing current device: %i", current_firing_device_ );

        _FiringDeviceActionService service;

        switch( current_firing_device_ )
        {
        case 0:
            _Shooter1ServiceClient::callService( service );
            break;
        case 1:
            _Shooter2ServiceClient::callService( service );
            break;
        case 2:
            _Dropper1ServiceClient::callService( service );
            break;
        case 3:
            _Dropper2ServiceClient::callService( service );
            break;
        }
    }

    QUICKDEV_SPIN_FIRST()
    {
        initPolicies
        <
            _JoystickPolicy,
            _Shooter1ServiceClient,
            _Shooter2ServiceClient,
            _Dropper1ServiceClient,
            _Dropper2ServiceClient
        >
        (
            "enable_key_ids", true, // enable keys with IDs appended for all policies in this group
            "robot_name_param", std::string( "seabee3" ),
            "service_name_param0", std::string( "/seabee3/shooter1" ),
            "service_name_param1", std::string( "/seabee3/shooter2" ),
            "service_name_param2", std::string( "/seabee3/dropper1" ),
            "service_name_param3", std::string( "/seabee3/dropper2" )
        );

        // initialize any remaining policies
        initPolicies<quickdev::policy::ALL>();
    }

    QUICKDEV_SPIN_ONCE()
    {
        _JoystickPolicy::update();
    }
};

#endif // SEABEE3TELEOP_SEABEE3TELEOPNODE_H_
