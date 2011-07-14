/*******************************************************************************
 *
 *      seabee3_stabilizer
 * 
 *      Copyright (c) 2011, John O'Hollaren
 *      All rights reserved.
 *
 *      Redistribution and use in source and binary forms, with or without
 *      modification, are permitted provided that the following conditions are
 *      met:
 *      
 *      * Redistributions of source code must retain the above copyright
 *        notice, this list of conditions and the following disclaimer.
 *      * Redistributions in binary form must reproduce the above
 *        copyright notice, this list of conditions and the following disclaimer
 *        in the documentation and/or other materials provided with the
 *        distribution.
 *      * Neither the name of "seabee3_stabilizer-RelWithDebInfo@seabee3_stabilizer" nor the names of its
 *        contributors may be used to endorse or promote products derived from
 *        this software without specific prior written permission.
 *      
 *      THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *      "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *      LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *      A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *      OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *      SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *      LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *      DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *      THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *      (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *      OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *******************************************************************************/

#ifndef SEABEE3_STABILIZER_H_
#define SEABEE3_STABILIZER_H_

#include <seabee3_driver_base/MotorCntl.h>
#include <xsens_node/Imu.h>
#include <base_node/base_node.h>
#include <common_utils/math.h>

// the IMU may be tilted inside of seabee so that 0.0 degrees isn't
// the nominal ideal pitch value, this constant can adjust that
#define NOMINAL_PITCH             0.0

typedef unsigned int _DimType;
typedef BaseNode<> _BaseNode;
typedef xsens_node::Imu _ImuMsgType;
typedef seabee3_driver_base::MotorCntl _MotorCntlMsgType;

class Seabee3Stabilizer : public _BaseNode
{
public:
	ros::Subscriber imu_sub_;
	ros::Subscriber motor_cntl_sub_;
	ros::Publisher motor_cntl_pub_;

  float delta_pitch;
  float scale_factor; 

  int motor1_value_scaled;
  int motor3_value_scaled;

	boost::mutex imu_msg_mutex_;
	_ImuMsgType last_imu_msg_;

	Seabee3Stabilizer( ros::NodeHandle & nh ) : _BaseNode( nh )
	{
		imu_sub_ = nh_local_.subscribe( "imu_data", 1, &Seabee3Stabilizer::imuCB, this );
		motor_cntl_sub_ = nh_local_.subscribe( "motor_cntl_raw", 1, &Seabee3Stabilizer::motorCntlCB, this );
		motor_cntl_pub_ = nh_local_.advertise<_MotorCntlMsgType>( "motor_cntl", 1 );
	}

	// store last IMU message; this callback is finished
	void imuCB( const _ImuMsgType::ConstPtr & imu_msg )
	{
		if( !imu_msg_mutex_.try_lock() ) return;

		last_imu_msg_ = *imu_msg;

		imu_msg_mutex_.unlock();
    
	}

	// use last imu message to scale motor values
	void motorCntlCB( const _MotorCntlMsgType::ConstPtr & motor_cntl_msg )
	{
		_MotorCntlMsgType::Ptr new_motor_cntl_msg( new _MotorCntlMsgType );

    // find difference between current pitch and nominal pitch 
		imu_msg_mutex_.lock();
    delta_pitch = abs( math_utils::radToDeg( last_imu_msg_.ori.y ) - NOMINAL_PITCH ); 
		imu_msg_mutex_.unlock();

    // we want the sub to stop if pitch is > 90. if its 91, we still want it to stop
    // so make 90 the hard limit
    if (delta_pitch > 90.0 )
      delta_pitch = 90.0;

    // slow down seabee based on how far off it is pitching
    // if pitch is 0 degrees off from the nominal value, just pass the velocity value through
    // if pitch is 90 degrees off from the nominal value, seabee must stop completely
    // for values in between 0 and 90, scale the speed down linearly
    scale_factor = ( 1 - delta_pitch / 90.0 );

    // create new forward motor commands based on scale factor
    // the forward motors are 1 and 3
    motor1_value_scaled = round( motor_cntl_msg->motors[1] * scale_factor );
    motor3_value_scaled = round( motor_cntl_msg->motors[3] * scale_factor );

    // create new array of motor commands 
    new_motor_cntl_msg->motors[0] = motor_cntl_msg->motors[0]; 
    new_motor_cntl_msg->motors[1] = motor1_value_scaled;
    new_motor_cntl_msg->motors[2] = motor_cntl_msg->motors[2]; 
    new_motor_cntl_msg->motors[3] = motor3_value_scaled;
    new_motor_cntl_msg->motors[4] = motor_cntl_msg->motors[4]; 
    new_motor_cntl_msg->motors[5] = motor_cntl_msg->motors[5]; 
    new_motor_cntl_msg->motors[6] = motor_cntl_msg->motors[6]; 
    new_motor_cntl_msg->motors[7] = motor_cntl_msg->motors[7]; 
    new_motor_cntl_msg->motors[8] = motor_cntl_msg->motors[8]; 

    // now pass the mask through the exact same as it was before
    new_motor_cntl_msg->mask[0] = motor_cntl_msg->mask[0];
    new_motor_cntl_msg->mask[1] = motor_cntl_msg->mask[1];
    new_motor_cntl_msg->mask[2] = motor_cntl_msg->mask[2];
    new_motor_cntl_msg->mask[3] = motor_cntl_msg->mask[3];
    new_motor_cntl_msg->mask[4] = motor_cntl_msg->mask[4];
    new_motor_cntl_msg->mask[5] = motor_cntl_msg->mask[5];
    new_motor_cntl_msg->mask[6] = motor_cntl_msg->mask[6];
    new_motor_cntl_msg->mask[7] = motor_cntl_msg->mask[7];
    new_motor_cntl_msg->mask[8] = motor_cntl_msg->mask[8];

//    ROS_INFO("\n\n\tCurrent Pitch: \t\t\t%f Degrees\n\tThruster Multiplier: \t\t%f Percent\n\tMotor1 Original Command: \t%i\n\tMotor3 Original Command: \t%i\n\tMotor1 Updated Command: \t%i\n\tMotor3 Updated Command: \t%i\n\n", last_imu_msg_->ori.y, scale_factor * 100, motor_cntl_msg->motors[1], motor_cntl_msg->motors[3], motor1_value_scaled, motor3_value_scaled); 

    // now publish these new motor control thrust values 
		motor_cntl_pub_.publish( new_motor_cntl_msg );
	}
};

#endif /* SEABEE3_STABILIZER_H_ */
