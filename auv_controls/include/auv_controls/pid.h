/***************************************************************************
 *  include/auv_controls/pid.h
 *  --------------------
 *
 *  Copyright (c) 2013, Dylan Foster
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
 *  * Neither the name of USC AUV nor the names of its
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

/// CPP11
#include <functional>

/// ROS
#include <ros/ros.h>

/// dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <auv_controls/PIDConfig.h>

#include <auv_controls/FeedbackLoop.h>

#include <uscauv_common/simple_math.h>

namespace uscauv
{

class ReconfigurablePIDSettings
{
 private:
  typedef auv_controls::PIDConfig _PIDConfig;
  typedef dynamic_reconfigure::Server<_PIDConfig> _PIDReconfigureServer;

 public:
  _PIDConfig config_;
  std::string name_;

 private:
  
  ros::NodeHandle nh_rel_, nh_pid_;
  std::shared_ptr<_PIDReconfigureServer> reconfigure_server_;
  
  std::function<void()> settings_changed_callback_;
  
 public:
  /// TODO: Modify so that ROS doesn't need to be running when the class is constructed
 ReconfigurablePIDSettings():
  nh_rel_( "~" )
  {
  }

  void init(std::string const & name)
  {
    name_ = name;

    /// Reconfigure server will resolve namespaces relative to ~/ns/name
    nh_pid_ = ros::NodeHandle( nh_rel_, name_ );
    
    reconfigure_server_ = std::make_shared<_PIDReconfigureServer>( nh_pid_ );

    _PIDReconfigureServer::CallbackType reconfigure_callback;
    reconfigure_callback = std::bind(&ReconfigurablePIDSettings::reconfigureCallback, this,
    				     std::placeholders::_1, std::placeholders::_2);

    reconfigure_server_->setCallback( reconfigure_callback );
    
    ROS_INFO("Created PID reconfigure server [ %s ]." , name_.c_str() );
    
 }

  void reconfigureCallback( _PIDConfig & config, uint32_t level )
  {
    config_ = config;

    if ( settings_changed_callback_ )
      {
	try
	  {
	    settings_changed_callback_();
	  }
	catch(const std::bad_function_call & ex)
	  {
	    ROS_ERROR_STREAM( "Reconfigurable PID settings callback failed [ " << ex.what() << " ]." );
	  }
      }
    
    return;
  }
  
  void registerCallback( std::function<void()> const & callback )
  {
    settings_changed_callback_ = callback;
    return;
  }
  
};

class PID1D
{
 private:
  double setpoint_, integral_term_, observed_value_, last_error_;

  ros::Time last_update_time_;
  std::string name_;

  ReconfigurablePIDSettings settings_;

 private:
  typedef auv_controls::FeedbackLoop _FeedbackLoopMsg;
  
  /// ROS interfaces
  ros::NodeHandle nh_rel_, nh_pid_;

  ros::Publisher feedback_pub_;
  
 public:

 PID1D():
  setpoint_( 0.0f ),
  integral_term_( 0.0f ),
  observed_value_( 0.0f ),
  last_error_( 0.0f ),
  nh_rel_("~")
    {
      
    }
 
  void init(std::string const & name, std::string const & ns = "pid")
  {
    name_ = ns + "/" + name;
    
    /// nh_pid will resolve namespaces relative to ~/ns/name
    nh_pid_ = ros::NodeHandle( nh_rel_, name_ );
    
    settings_.init( name_ );

    std::function<void()> reconfigure_cb = std::bind(&PID1D::reconfigureCallback, this);

    settings_.registerCallback( reconfigure_cb );
    
    feedback_pub_ = nh_pid_.advertise<_FeedbackLoopMsg>( "feedback", 1 );
    
    ROS_INFO("Created PID publisher [ %s ].", (name_ + "/feedback").c_str());

    last_update_time_ = ros::Time::now();

    return;
  }

  void setSetpoint(double const & setpoint)
  {
    setpoint_ = setpoint;
    return;
  }
  
  void setObserved(double const & observed)
  {
    observed_value_ = observed;
    return;
  }
  
  double update()
  {
    auv_controls::PIDConfig const & config = settings_.config_;
    
    ros::Time now = ros::Time::now();
    
    double dt = (now - last_update_time_).toSec();

    /// error terms
    double error;
    if( config.use_mod )
      error = -uscauv::ring_difference( setpoint_, observed_value_, config.mod_val );
    else
      error = setpoint_ - observed_value_;
    double dedt = (error - last_error_) / dt;
    integral_term_ += error * dt;
    last_error_ = error;
    
    double const & p = config.p_gain;
    double const & i = config.i_gain;
    double const & d = config.d_gain;

    double output = p*error + i*integral_term_ + d*dedt;
        
    last_update_time_ = now;
      
    publishLoop(setpoint_, observed_value_, output);

    return output;
  }

  void reconfigureCallback()
  {
    ROS_INFO("Resetting integral term...");

    integral_term_ = 0.0f;
  }
  
 private:
  void publishLoop(double const & x, double const & y, double const & e)
    {
      boost::shared_ptr<_FeedbackLoopMsg> msg( new _FeedbackLoopMsg );
      
      msg->x = x;
      msg->y = y;
      msg->e = e;
      
      feedback_pub_.publish(msg);
    }
   
};

} //uscauv
