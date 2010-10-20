/*******************************************************************************
 *
 *      base_teleop
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

#ifndef BASE_TELEOP_H_
#define BASE_TELEOP_H_

#include <base_node/base_node.h>
#include <geometry_msgs/Twist.h>
#include <joy/Joy.h>

class BaseTeleop: public BaseNode
{

protected:
	geometry_msgs::Twist cmd_vel_;
	ros::Publisher cmd_vel_pub_;
	ros::Subscriber joy_sub_;

public:
	BaseTeleop( ros::NodeHandle & nh, std::string topic_name = "cmd_vel" );
	~BaseTeleop();

protected:
	virtual void joyCB( const joy::Joy::ConstPtr& joy );
	double applyDeadZone( double value, double dead_radius_max = 0.15 );

private:
	void joyCB_0( const joy::Joy::ConstPtr& joy );

};

BaseTeleop::BaseTeleop( ros::NodeHandle & nh, std::string topic_name ) :
	BaseNode( nh, 1 )
{
	joy_sub_ = nh.subscribe( nh.resolveName( "/joy" ), 1, &BaseTeleop::joyCB_0, this );
	cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist> ( topic_name, 1 );
}

BaseTeleop::~BaseTeleop()
{
	//
}

// virtual
void BaseTeleop::joyCB( const joy::Joy::ConstPtr& joy )
{
	//
}

void BaseTeleop::joyCB_0( const joy::Joy::ConstPtr& joy )
{
	joyCB( joy );
}

double BaseTeleop::applyDeadZone( double value, double dead_radius_max )
{
	return fabs( value ) < dead_radius_max ? 0.0 : value;
}

#endif /* BASE_TELEOP_H_ */
