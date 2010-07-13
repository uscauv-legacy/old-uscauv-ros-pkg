/*******************************************************************************
 *
 *      seabee3_teleop
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
 *      * Neither the name of the USC Underwater Robotics Team nor the names of its
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

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <joy/Joy.h>
#include <seabee3_beestem/BeeStem3_driver.h>
#include <seabee3_driver_base/FiringDeviceAction.h>

ros::Publisher * cmd_vel_pub;
int speed, strafe, surface, dive, heading, roll, f_dev_inc, f_dev_dec, fire_dev, currentDevice = 0;
double speed_s, strafe_s, surface_s, dive_s, heading_s, roll_s;

ros::ServiceClient shooter_srv, dropper1_srv, dropper2_srv;
seabee3_driver_base::FiringDeviceAction device_action;
bool buttonActionBusy = false;
int buttonActionId = -1;

double applyDeadZone (double value)
{
	return fabs(value) < 0.15 ? 0.0 : value;
}

void fireCurrentDevice (bool reset = false)
{
	int mode = reset ? 1 : 0;
	device_action.request.Req = mode;
	switch(currentDevice)
	{
	case BeeStem3Driver::FiringDeviceID::Shooter:
		shooter_srv.call(device_action);
		break;
	case BeeStem3Driver::FiringDeviceID::DropperStage1:
		dropper1_srv.call(device_action);
		break;
	case BeeStem3Driver::FiringDeviceID::DropperStage2:
		dropper2_srv.call(device_action);
		break;
	}
}

void updateFiringDevices(int action = -1)
{
	bool fire = false;
	switch(action)
	{
	case 0:
		currentDevice ++;
		break;
	case 1:
		currentDevice --;
		break;
	case 2:
		fire = true;
		break;
	}
	
	currentDevice = currentDevice > BeeStem3Driver::FiringDeviceID::DropperStage2 ? 0 : currentDevice < 0 ? BeeStem3Driver::FiringDeviceID::DropperStage2 : currentDevice;
	
	ROS_INFO("current device %d", currentDevice);
	
	if(fire)
	{
		fireCurrentDevice();
	}
}

void joyCallback(const joy::Joy::ConstPtr& joy)
{
	if(buttonActionBusy)
	{
		if(joy->buttons[buttonActionId] == 0)
		{
			buttonActionBusy = false;
		}
	}
	else
	{
		buttonActionBusy = joy->buttons[f_dev_inc] == 1 || joy->buttons[f_dev_dec] == 1 || joy->buttons[fire_dev] == 1;
		buttonActionId = joy->buttons[f_dev_inc] == 1 ? f_dev_inc : buttonActionId;
		buttonActionId = joy->buttons[f_dev_dec] == 1 ? f_dev_dec : buttonActionId;
		buttonActionId = joy->buttons[fire_dev] == 1 ? fire_dev : buttonActionId;
		if(buttonActionBusy)
		{
			updateFiringDevices(joy->buttons[f_dev_inc] == 1 ? 0 : joy->buttons[f_dev_dec] == 1 ? 1 : joy->buttons[fire_dev] == 1 ? 2 : -1);
		}
	}
	
	geometry_msgs::Twist cmd_vel;
	
	double joy_speed = 		applyDeadZone( (double)(joy->axes[speed]) );
	double joy_strafe = 	applyDeadZone( (double)(joy->axes[strafe]) );
	double joy_dive = 		applyDeadZone( (double)(joy->axes[dive]) );
	double joy_surface = 	applyDeadZone( (double)(joy->axes[surface]) );
	double joy_heading = 	applyDeadZone( (double)(joy->axes[heading]) );
	double joy_roll = 		applyDeadZone( (double)(joy->axes[roll]) );
	
	cmd_vel.linear.x = speed_s * joy_speed; //speed
	cmd_vel.linear.y = strafe_s * joy_strafe; //strafe
	cmd_vel.linear.z = 0.5 * (float)(dive_s * joy_dive - surface_s * joy_surface); //dive - surface
	
	cmd_vel.angular.x = roll_s * joy_roll; //roll
	cmd_vel.angular.y = 0;
	cmd_vel.angular.z = heading_s * joy_heading; //heading
	
	cmd_vel_pub->publish(cmd_vel);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "seabee3_teleop");
	ros::NodeHandle n("~");
	
	n.param("speed", speed, 0);
	n.param("strafe", strafe, 1);
	n.param("surface", surface, 2);
	n.param("dive", dive, 5);
	n.param("heading", heading, 4);
	n.param("roll", roll, 3);
	n.param("next_firing_device", f_dev_inc, 0);
	n.param("prev_firing_device", f_dev_dec, 1);
	n.param("fire_defice", fire_dev, 5);
	
	n.param("speed_scale", speed_s, 1.0);
	n.param("strafe_scale", strafe_s, 1.0);
	n.param("surface_scale", surface_s, 1.0);
	n.param("dive_scale", dive_s, 1.0);
	n.param("heading_scale", heading_s, 1.0);
	n.param("roll_scale", roll_s, 1.0);
	
	ros::Subscriber joy_sub = n.subscribe("/joy", 1, joyCallback);
	
	shooter_srv = n.serviceClient<seabee3_driver_base::FiringDeviceAction>("/seabee3/ShooterAction");
	dropper1_srv = n.serviceClient<seabee3_driver_base::FiringDeviceAction>("/seabee3/Dropper1Action");
	dropper2_srv = n.serviceClient<seabee3_driver_base::FiringDeviceAction>("/seabee3/Dropper2Action");
	
	cmd_vel_pub = new ros::Publisher;
	*cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/seabee3/cmd_vel", 1);
	
	ros::spin();
	return 0;
}


