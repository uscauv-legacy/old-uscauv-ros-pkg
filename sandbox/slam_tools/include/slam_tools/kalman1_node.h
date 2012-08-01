/***************************************************************************
 *  include/slam_tools/kalman1_node.h
 *  --------------------
 *
 *  Copyright (c) 2011, nalyd
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

#ifndef SLAMTOOLS_KALMAN1NODE_H_
#define SLAMTOOLS_KALMAN1NODE_H_

#include <quickdev/node.h>
#include <quickdev/multi_subscriber.h>
#include <quickdev/multi_publisher.h>
#include <slam_tools/MeasurementWithVariance.h>

#define INITIAL_MEAN 0
#define INITIAL_VARIANCE 100
#define MEASUREMENT_TOPIC "measurement"
#define CONTROL_TOPIC "control"
#define STATE_TOPIC "estimate"



QUICKDEV_DECLARE_NODE( Kalman1 )

// Declare a class called Kalman1Node
QUICKDEV_DECLARE_NODE_CLASS( Kalman1 )
{
  typedef slam_tools::MeasurementWithVariance _KalmanDataMsg;

  //  ros::Publisher state_estimate_pub_;
  ros::MultiSubscriber<> multi_sub_;
  ros::MultiPublisher<> multi_pub_;
  
  // State variables
  double state_mean_, state_var_;
  std::string control_topic_, measurement_topic_, state_topic_;
  

  QUICKDEV_DECLARE_NODE_CONSTRUCTOR( Kalman1 ), state_mean_(INITIAL_MEAN), state_var_(INITIAL_VARIANCE){}
    
  QUICKDEV_SPIN_FIRST()
    {
      initPolicies<quickdev::policy::ALL>();

      QUICKDEV_GET_RUNABLE_NODEHANDLE( nh_rel );

      // Get topics for state, control signal, and measurement signal from parameter, otherwise use defaults
      nh_rel.param<std::string>("control_topic", control_topic_, CONTROL_TOPIC);
      nh_rel.param<std::string>("measurement_topic", measurement_topic_, MEASUREMENT_TOPIC);
      nh_rel.param<std::string>("state_topic", state_topic_, STATE_TOPIC);
      
      // Get initial state from parameter server
      nh_rel.param<double>("initial_mean", state_mean_, INITIAL_MEAN);
      nh_rel.param<double>("initial_variance", state_var_, INITIAL_VARIANCE);


      multi_sub_.addSubscriber( nh_rel, control_topic_, &Kalman1Node::newControlCB, this );
      multi_sub_.addSubscriber( nh_rel, measurement_topic_, &Kalman1Node::newMeasurementCB, this );
      
      multi_pub_.addPublishers<_KalmanDataMsg>(nh_rel, {state_topic_});
    }


  QUICKDEV_SPIN_ONCE(){}
  

  QUICKDEV_DECLARE_MESSAGE_CALLBACK(newControlCB, _KalmanDataMsg)
    {
      // Update the prediction of the next state using the a new control signal
      predictionUpdate(msg->value, msg->variance);
      
      _KalmanDataMsg state_update_msg;
	    
      state_update_msg.value = state_mean_;
      
      state_update_msg.variance = state_var_;

      // Publish the updated state
      //state_estimate_pub_.publish(state_update_msg);
      multi_pub_.publish(STATE_TOPIC, state_update_msg);
      
    }
  

  QUICKDEV_DECLARE_MESSAGE_CALLBACK(newMeasurementCB, _KalmanDataMsg)
    {
      // Update the state using the new measurement
      measurementUpdate(msg->value, msg->variance);
      
      _KalmanDataMsg state_update_msg;
      
      state_update_msg.value = state_mean_;
      
      state_update_msg.variance = state_var_;

      // Publish the updated state
      //state_estimate_pub_.publish(state_update_msg);
      multi_pub_.publish(STATE_TOPIC, state_update_msg);
    }


  // Update our prediction of the state based on new control signals
  void predictionUpdate(double control, double control_var)
  {
    // Predict that the state will be exactly what it would be if there were no process noise
    state_mean_ += control;

    // Uncertainty is increased by the process variance of the control signal
    state_var_ += control_var;
  }
    

  // Use a new measurement and its variance to update the estimate of the state
  void measurementUpdate(double measurement, double measurement_var)
  {
    // Compute the Kalman filter gain
    double kalman_gain = state_var_/(state_var_ + measurement_var);
      
    // Update the state to account for the new measurementxs
    state_mean_ += kalman_gain*(measurement - state_mean_ );
    state_var_ *= (1 - kalman_gain);
  }

};

#endif // SLAMTOOLS_KALMAN1NODE_H_
