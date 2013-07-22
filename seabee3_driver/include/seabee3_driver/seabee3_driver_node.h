/***************************************************************************
 *  include/seabee3_driver/seabee3_driver_node.h
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

#ifndef SEABEE3DRIVER_SEABEE3DRIVERNODE_H_
#define SEABEE3DRIVER_SEABEE3DRIVERNODE_H_

#include <cstdlib>

#include <quickdev/node.h>

// policies
#include <quickdev/robot_driver_policy.h>
#include <quickdev/service_server_policy.h>
#include <quickdev/reconfigure_policy.h>

// objects
#include <seabee3_driver/bee_stem3_driver.h>

// msgs
#include <seabee3_msgs/MotorVals.h>
#include <seabee3_msgs/Depth.h>
#include <seabee3_msgs/KillSwitch.h>
#include <seabee3_msgs/Pressure.h>

// service
#include <seabee3_msgs/FiringDeviceAction.h>
#include <seabee3_msgs/CalibrateDouble.h>

// cfgs
#include <seabee3_driver/FakeSeabeeConfig.h>

typedef seabee3_msgs::MotorVals _MotorValsMsg;
typedef seabee3_msgs::Depth _DepthMsg;
typedef seabee3_msgs::KillSwitch _KillSwitchMsg;
typedef seabee3_msgs::Pressure _PressureMsg;

typedef seabee3_msgs::FiringDeviceAction _FiringDeviceActionService;
typedef seabee3_msgs::CalibrateDouble _CalibrateSurfacePressureService;

typedef seabee3_driver::FakeSeabeeConfig _FakeSeabeeCfg;

typedef quickdev::RobotDriverPolicy<_MotorValsMsg> _RobotDriver;
typedef quickdev::ServiceServerPolicy<_FiringDeviceActionService, 0> _Shooter1ServiceServer;
typedef quickdev::ServiceServerPolicy<_FiringDeviceActionService, 1> _Shooter2ServiceServer;
typedef quickdev::ServiceServerPolicy<_FiringDeviceActionService, 2> _Dropper1ServiceServer;
typedef quickdev::ServiceServerPolicy<_FiringDeviceActionService, 3> _Dropper2ServiceServer;
typedef quickdev::ServiceServerPolicy<_CalibrateSurfacePressureService, 4> _CalibrateSurfacePressureServiceServer;
typedef quickdev::ReconfigurePolicy<_FakeSeabeeCfg> _FakeSeabeeLiveParams;
typedef quickdev::TfTranceiverPolicy _TfTranceiverPolicy;

using namespace seabee3_common;

QUICKDEV_DECLARE_NODE( Seabee3Driver, _RobotDriver, _Shooter1ServiceServer, _Shooter2ServiceServer, _Dropper1ServiceServer, _Dropper2ServiceServer, _CalibrateSurfacePressureServiceServer, _FakeSeabeeLiveParams )

QUICKDEV_DECLARE_NODE_CLASS( Seabee3Driver )
{
    BeeStem3Driver bee_stem3_driver_;
    std::array<int, movement::NUM_MOTOR_CONTROLLERS> motor_dirs_;
    double surface_pressure_;

    QUICKDEV_DECLARE_NODE_CONSTRUCTOR( Seabee3Driver )
    {
        //
    }

    QUICKDEV_SPIN_FIRST()
    {
        QUICKDEV_GET_RUNABLE_NODEHANDLE( nh_rel );
	
	/// TODO: Change default value to 895, figure out how to properly resolve param namespace
	surface_pressure_ = ros::ParamReader<double, 1>::readParam( nh_rel, "surface_pressure", 666);
		
	motor_dirs_[movement::MotorControllerIDs::FWD_RIGHT_THRUSTER] =     -1;
        motor_dirs_[movement::MotorControllerIDs::FWD_LEFT_THRUSTER] =      -1;
        motor_dirs_[movement::MotorControllerIDs::DEPTH_FRONT_THRUSTER] =   -1;
        motor_dirs_[movement::MotorControllerIDs::DEPTH_BACK_THRUSTER] =     1;
        motor_dirs_[movement::MotorControllerIDs::STRAFE_TOP_THRUSTER] =    -1;
        motor_dirs_[movement::MotorControllerIDs::STRAFE_BOTTOM_THRUSTER] = -1;

        _RobotDriver::registerCallback( quickdev::auto_bind( &Seabee3DriverNode::motorValsCB, this ) );
        _Shooter1ServiceServer::registerCallback( quickdev::auto_bind( &Seabee3DriverNode::shooter1CB, this ) );
        _Shooter2ServiceServer::registerCallback( quickdev::auto_bind( &Seabee3DriverNode::shooter2CB, this ) );
        _Dropper1ServiceServer::registerCallback( quickdev::auto_bind( &Seabee3DriverNode::dropper1CB, this ) );
        _Dropper2ServiceServer::registerCallback( quickdev::auto_bind( &Seabee3DriverNode::dropper2CB, this ) );
	_CalibrateSurfacePressureServiceServer::registerCallback( quickdev::auto_bind( &Seabee3DriverNode::calibrateSurfacePressureCB, this) );
        _FakeSeabeeLiveParams::registerCallback( quickdev::auto_bind( &Seabee3DriverNode::reconfigureCB, this ) );

        initPolicies
        <
            _RobotDriver,
            _Shooter1ServiceServer,
            _Shooter2ServiceServer,
            _Dropper1ServiceServer,
	    _Dropper2ServiceServer,
	    _CalibrateSurfacePressureServiceServer
        >
        (
            "enable_key_ids", true, // enable keys with IDs appended for all policies in this group
            "robot_name_param", std::string( "seabee3" ),
            "service_name_param0", std::string( "/seabee3/shooter1" ),
            "service_name_param1", std::string( "/seabee3/shooter2" ),
            "service_name_param2", std::string( "/seabee3/dropper1" ),
            "service_name_param3", std::string( "/seabee3/dropper2" ),
	    "service_name_param4", std::string( "/seabee3/calibrate_surface_pressure" )
        );

        // initialize any remaining policies
        initPolicies<quickdev::policy::ALL>();

        multi_pub_.addPublishers
        <
            _DepthMsg,
            _KillSwitchMsg,
            _PressureMsg,
            _PressureMsg
        >
        (
            nh_rel,
            {
                "/robot/sensors/depth",
                "/robot/sensors/kill_switch",
                "/robot/sensors/internal_pressure",
                "/robot/sensors/external_pressure"
            }
        );
/*
        // assign a default value to the slot for all thrusters
        std::map<std::string, int> motor_dirs_map;
        for( size_t i = 0; i < movement::NUM_MOTOR_CONTROLLERS; ++i )
        {
            std::stringstream ss;
            ss << i;
            motor_dirs_map[ss.str()] = 1;
        }

        // read in any user overrides to the thruster directions
        auto motor_dirs_param = quickdev::ParamReader::readParam<std::map<std::string, int> >( nh_rel, "motor_dirs" );

        // overwrite default values with user values
        for( auto motor_dirs_it = motor_dirs_param.begin(); motor_dirs_it != motor_dirs_param.end(); ++motor_dirs_it )
        {
            motor_dirs_map[motor_dirs_it->first] = motor_dirs_it->second;
        }

        // store final values in an array
        for( size_t i = 0; i < motor_dirs_.size(); ++i )
        {
            std::stringstream ss;
            ss << i;
            auto motor_dirs_it = motor_dirs_map.find( ss.str() );

            if( motor_dirs_it != motor_dirs_map.end() ) motor_dirs_[i] = motor_dirs_it->second;
        }
*/
    }

    inline double getDepthFromPressure( int const & observed_pressure ) const
    {
      /* return ( observed_pressure - config_.surface_pressure ) / config_.montalbos_per_meter; */
      return ( observed_pressure - surface_pressure_ ) / config_.montalbos_per_meter;
    }

    QUICKDEV_SPIN_ONCE()
    {
        _PressureMsg intl_pressure_msg;
        _PressureMsg extl_pressure_msg;
        _DepthMsg depth_msg;
        _KillSwitchMsg kill_switch_msg;
	
        if( config_.simulate )
        {
            intl_pressure_msg.value = config_.internal_pressure;
            extl_pressure_msg.value = config_.external_pressure;
            kill_switch_msg.is_killed = config_.is_killed;
        }
        else
        {
            bee_stem3_driver_.readPressure( intl_pressure_msg.value, extl_pressure_msg.value );
            bee_stem3_driver_.readKillSwitch( kill_switch_msg.is_killed );
        }

        auto const depth = getDepthFromPressure( extl_pressure_msg.value );

        depth_msg.value = depth;

        multi_pub_.publish(
            "/robot/sensors/depth", depth_msg,
            "/robot/sensors/kill_switch", kill_switch_msg,
            "/robot/sensors/internal_pressure", intl_pressure_msg,
            "/robot/sensors/external_pressure", extl_pressure_msg
        );

        _TfTranceiverPolicy::publishTransform( tf::Transform( tf::Quaternion( 0, 0, 0, 1 ), tf::Vector3( 0, 0, -depth ) ), "/world", "/robot/sensors/depth" );
    }

    bool executeFiringDeviceAction( _FiringDeviceActionService::Request &req,
                                    _FiringDeviceActionService::Response &res,
                                    int const & device_id )
    {
        if( !bee_stem3_driver_.connected() ) return false;

        bool device_status = bee_stem3_driver_.getDeviceStatus( device_id );
        switch ( req.action )
        {
        case _FiringDeviceActionService::Request::CHECK_STATUS:
            break;
        case _FiringDeviceActionService::Request::RESET_STATUS:
            device_status = true;
            break;
        case _FiringDeviceActionService::Request::FIRE:
            bee_stem3_driver_.fireDevice( device_id );
            break;
        }
        res.is_loaded = device_status;
        return true;
    }

    double runSurfacePressureCalibration(const unsigned int & num_samples)
    {
      ROS_INFO( "Aquiring pressure samples..." );
      int internal_pressure, external_pressure;
      double surface_pressure = 0.0;
      
      for(unsigned int i = 0; i < num_samples && QUICKDEV_GET_RUNABLE_POLICY()::running(); ++i)
	{
	  bee_stem3_driver_.readPressure(internal_pressure, external_pressure);
	  surface_pressure += external_pressure;
	    
	  ros::spinOnce();
	  QUICKDEV_GET_RUNABLE_POLICY()::getLoopRate()->sleep();
	}
      
      return surface_pressure / double(num_samples);
    }
    
    QUICKDEV_DECLARE_MESSAGE_CALLBACK( motorValsCB, _MotorValsMsg )
    {
        if( config_.simulate && config_.is_killed ) return;

        auto const & motors = msg->motors;
        auto const & mask = msg->mask;
        for( size_t i = 0; i < motors.size(); ++i )
        {
	  if( config_.override_kill_switch )
            {
	      bee_stem3_driver_.setThruster( i, 0 );
	      continue;
            }
	  if( mask[i] )
            {
	      auto motor_value = motor_dirs_[i] * motors[i];
	      if( motor_value != 0 && abs( motor_value ) < config_.motor_speed_floor ) motor_value = 0;
	      
	      if( config_.simulate )
		{
		  PRINT_INFO( "Setting motor id %zu to %d", i, motor_value );
                }
	      else
                {
		  bee_stem3_driver_.setThruster( i, motor_value );
                }
            }
        }
    }

    QUICKDEV_DECLARE_SERVICE_CALLBACK( shooter1CB, _FiringDeviceActionService )
    {
        if( config_.simulate ) PRINT_INFO( "Firing first torpedo!" );
        else return executeFiringDeviceAction( request, response, movement::FiringDeviceIDs::SHOOTER1 );

        return true;
    }

    QUICKDEV_DECLARE_SERVICE_CALLBACK( shooter2CB, _FiringDeviceActionService )
    {
        if( config_.simulate ) PRINT_INFO( "Firing second torpedo!" );
        else return executeFiringDeviceAction( request, response, movement::FiringDeviceIDs::SHOOTER2 );

        return true;
    }

    QUICKDEV_DECLARE_SERVICE_CALLBACK( dropper1CB, _FiringDeviceActionService )
    {
        if( config_.simulate ) PRINT_INFO( "Dropping first marker!" );
        else return executeFiringDeviceAction( request, response, movement::FiringDeviceIDs::DROPPER_STAGE1 );

        return true;
    }

    QUICKDEV_DECLARE_SERVICE_CALLBACK( dropper2CB, _FiringDeviceActionService )
    {
        if( config_.simulate ) PRINT_INFO( "Dropping second marker!" );
        else return executeFiringDeviceAction( request, response, movement::FiringDeviceIDs::DROPPER_STAGE2 );

        return true;
    }

    /// Return value tells client whether or not the data returned is valid
    QUICKDEV_DECLARE_SERVICE_CALLBACK2( calibrateSurfacePressureCB, _CalibrateSurfacePressureService, request, response )
      {
	ROS_INFO( "Running surface pressure calibration..." );

	if( config_.simulate)
	  {
	    ROS_WARN( "Calibrating surface pressure using simulated data..." );
	    surface_pressure_ = config_.surface_pressure;
	  }
	else
	  {
	    surface_pressure_ = runSurfacePressureCalibration(request.num_samples);
	  }
	response.calibration = surface_pressure_;
	/// TODO: Use node namespace for parameter
	ros::param::set("/seabee3_driver/surface_pressure", surface_pressure_);

	/// Write params to file
	if( !system(NULL) )
	  {
	    ROS_ERROR( "Unable to write surface pressure calibration to file (processor unavailable)." );
	    return false;
	  }
	else
	  {
	    /// TODO: Get namespace of node rather than hard-coding it
	    int system_return = system("rosparam dump `rospack find seabee3_driver`/params/surface_pressure.yaml /seabee3_driver/surface_pressure");
	    if(system_return)
	      {
		ROS_ERROR( "Failed to write surface calibration to file (system call returned with %d).", system_return );
		return false;
	      }
	    else 
	      {
		ROS_INFO( "Wrote calibration to file." );
		return true;
	      }
	  }
	
      }

    /// TODO: Set value of surface_pressure_ on callback
   /**
    * config refers to the _FakeSeabeeCfg passed as the argument to this function by the reconfigure server
    * The value of config_ is set to config immediately after this function is called
    */
    QUICKDEV_DECLARE_RECONFIGURE_CALLBACK( reconfigureCB, _FakeSeabeeCfg )
    {

        if( !config.simulate )
        {
            bee_stem3_driver_.connect( config.port );
            bee_stem3_driver_.shooter1_params_.trigger_time_ =  config.shooter1_trigger_time;
            bee_stem3_driver_.shooter1_params_.trigger_value_ = config.shooter1_trigger_value;
            bee_stem3_driver_.shooter2_params_.trigger_time_ =  config.shooter2_trigger_time;
            bee_stem3_driver_.shooter2_params_.trigger_value_ = config.shooter2_trigger_value;
            bee_stem3_driver_.dropper1_params_.trigger_time_ =  config.dropper1_trigger_time;
            bee_stem3_driver_.dropper1_params_.trigger_value_ = config.dropper1_trigger_value;
            bee_stem3_driver_.dropper2_params_.trigger_time_ =  config.dropper2_trigger_time;
            bee_stem3_driver_.dropper2_params_.trigger_value_ = config.dropper2_trigger_value;
        }

        motor_dirs_[movement::MotorControllerIDs::FWD_RIGHT_THRUSTER] =     config.fwd_right_thruster_dir;
        motor_dirs_[movement::MotorControllerIDs::FWD_LEFT_THRUSTER] =      config.fwd_left_thruster_dir;
        motor_dirs_[movement::MotorControllerIDs::DEPTH_FRONT_THRUSTER] =   config.depth_front_thruster_dir;
        motor_dirs_[movement::MotorControllerIDs::DEPTH_BACK_THRUSTER] =    config.depth_back_thruster_dir;
        motor_dirs_[movement::MotorControllerIDs::STRAFE_TOP_THRUSTER] =    config.strafe_top_thruster_dir;
        motor_dirs_[movement::MotorControllerIDs::STRAFE_BOTTOM_THRUSTER] = config.strafe_bottom_thruster_dir;
    }
};

#endif // SEABEE3DRIVER_SEABEE3DRIVERNODE_H_
