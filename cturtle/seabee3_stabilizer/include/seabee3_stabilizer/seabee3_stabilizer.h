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

	boost::mutex imu_msg_mutex_;
	_ImuMsgType::ConstPtr last_imu_msg_;

	Seabee3Stabilizer( ros::NodeHandle & nh ) : _BaseNode( nh )
	{
		imu_sub_ = nh_local_.subscribe( "/xsens/custom_data", 1, &Seabee3Stabilizer::imuCB, this );
		motor_cntl_sub_ = nh_local_.subscribe( "/seabee3/motor_cntl_raw", 1, &Seabee3Stabilizer::motorCntlCB, this );
		motor_cntl_pub_ = nh_local_.advertise<_MotorCntlMsgType>( "/seabee3/motor_cntl", 1 );
	}

	// store last IMU message; this callback is finished
	void imuCB( const _ImuMsgType::ConstPtr & imu_msg )
	{
		if( !imu_msg_mutex_.try_lock() ) return;

		last_imu_msg_ = imu_msg;

		imu_msg_mutex_.unlock();
	}

	// use last imu message to scale motor values
	void motorCntlCB( const _MotorCntlMsgType::ConstPtr & motor_cntl_msg )
	{
		_MotorCntlMsgType::Ptr new_motor_cntl_msg( new _MotorCntlMsgType );

		imu_msg_mutex_.lock();

		// TODO: read IMU values from last_imu_msg_ and calculate scalar for motor values here
		// TODO: to see the IMU and MotorCntl message definitions, type 'rosmsg show <package_name>/<message_name>' in a terminal; note: 'xsens_node/Imu' 'seabee3_driver_base/MotorCntl'

		imu_msg_mutex_.unlock();

		motor_cntl_pub_.publish( new_motor_cntl_msg );
	}
};

#endif /* SEABEE3_STABILIZER_H_ */
