/*******************************************************************************
 *
 *      base_robot_driver
 * 
 *      Copyright (c) 2010, Edward T. Kaszubski (ekaszubski@gmail.com)
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
 *      * Neither the name of "seabee3-ros-pkg" nor the names of its
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

#ifndef BASE_ROBOT_DRIVER_H_
#define BASE_ROBOT_DRIVER_H_

#include <base_tf_tranceiver/base_tf_tranceiver.h>
#include <timeout_monitor/timeout_monitor.h>
#include <common_utils/operators.h>

template<typename _ReconfigureType = BaseNodeTypes::_DefaultReconfigureType>
class BaseRobotDriver : public BaseTfTranceiver<_ReconfigureType>
{
protected:
	ros::Subscriber cmd_vel_sub_;
	geometry_msgs::Twist twist_cache_;
	double cmd_vel_timeout_;
	TimeoutMonitor cmd_vel_tm_;

public:
	BaseRobotDriver( ros::NodeHandle & nh, std::string topic_name = "cmd_vel") :
		BaseTfTranceiver<_ReconfigureType> ( nh )
	{
		this->nh_local_.param( "cmd_vel_timeout", cmd_vel_timeout_, 0.5 );

		cmd_vel_sub_ = nh.subscribe( nh.resolveName( topic_name ), 1, &BaseRobotDriver::cmdVelCB_0, this );
		cmd_vel_tm_.registerCallback( nh, &BaseRobotDriver::timeout_0, this, cmd_vel_timeout_ );
	}

	virtual ~BaseRobotDriver()
	{
		//
	}

protected:
	virtual void cmdVelCB( const geometry_msgs::TwistConstPtr & twist )
	{
		cmd_vel_tm_.keepAlive();
		twist_cache_ = *twist;
	}

	virtual void timeout()
	{
		twist_cache_ = geometry_msgs::Twist();
	}

private:
	void cmdVelCB_0( const geometry_msgs::TwistConstPtr & twist )
	{
		cmdVelCB( twist );
	}

	void timeout_0( const ros::TimerEvent & evt )
	{
		cmd_vel_tm_.stop();
		timeout();
	}
};

#endif /* BASE_ROBOT_DRIVER_H_ */
