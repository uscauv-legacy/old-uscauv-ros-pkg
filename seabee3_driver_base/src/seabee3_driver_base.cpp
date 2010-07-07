/*******************************************************************************
 *
 *      seabee3_driver_base
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
#include <seabee3_beestem/BeeStem3_driver.h>
#include <seabee3_driver_base/KillSwitch.h>
#include <seabee3_driver_base/Pressure.h>
#include <seabee3_driver_base/MotorCntl.h>
#include <seabee3_driver_base/FiringDeviceAction.h>

BeeStem3Driver * mDriver;
int usbIndex;

void motorCntlCallback(const seabee3_driver_base::MotorCntlConstPtr & msg)
{
	//cout << "setting motor values..." << endl;
	for (unsigned int i = 0; i < msg->motors.size(); i++)
	{
		// printf("%d|%d : ", i, msg->values[i]);
		if(msg->mask[i] == 1)
			mDriver->itsStem->setThruster(i, msg->motors[i]);
	}
	//cout << endl;
}

bool Dropper1ActionCB(seabee3_driver_base::FiringDeviceAction::Request &req, seabee3_driver_base::FiringDeviceAction::Response &res)
{
	if(req.Req == 1)
	{
		mDriver->dropper1_ready = true;
	}
	res.Resp = mDriver->dropper1_ready ? 1 : 0;
	if(req.Req != 1)
		mDriver->fireDevice(BeeStem3Driver::FiringDeviceID::DropperStage1);
	return true;
}

bool Dropper2ActionCB(seabee3_driver_base::FiringDeviceAction::Request &req, seabee3_driver_base::FiringDeviceAction::Response &res)
{
	if(req.Req == 1)
	{
		mDriver->dropper2_ready = true;
	}
	res.Resp = mDriver->dropper2_ready ? 1 : 0;
	if(req.Req != 1)
		mDriver->fireDevice(BeeStem3Driver::FiringDeviceID::DropperStage2);
	return true;
}

bool ShooterActionCB(seabee3_driver_base::FiringDeviceAction::Request &req, seabee3_driver_base::FiringDeviceAction::Response &res)
{
	if(req.Req == 1)
	{
		mDriver->shooter_ready = true;
	}
	res.Resp = mDriver->shooter_ready ? 1 : 0;
	if(req.Req != 1)
		mDriver->fireDevice(BeeStem3Driver::FiringDeviceID::Shooter);
	return true;
}

int main (int argc, char** argv)
{
	ros::init(argc, argv, "seabee3_driver_base");
	ros::NodeHandle n("~");
	
	n.param("usb_index", usbIndex, 0);
	
	ROS_INFO("constructing new driver instance");
	mDriver = new BeeStem3Driver(usbIndex);
	
	ROS_INFO("subscribing to MotorCntl");
	ros::Subscriber motor_cntl_sub = n.subscribe("/seabee3/MotorCntl", 100, motorCntlCallback);
	
	ros::Publisher intl_pressure_pub = n.advertise<seabee3_driver_base::Pressure>("/seabee3/IntlPressure", 100);
	ros::Publisher extl_pressure_pub = n.advertise<seabee3_driver_base::Pressure>("/seabee3/ExtlPressure", 100);
	ros::Publisher kill_switch_pub = n.advertise<seabee3_driver_base::KillSwitch>("/seabee3/KillSwitch", 100);
	
	ros::ServiceServer dropper1action_srv = n.advertiseService("/seabee3/Dropper1Action", Dropper1ActionCB);
	ros::ServiceServer dropper2action_srv = n.advertiseService("/seabee3/Dropper2Action", Dropper2ActionCB);
	ros::ServiceServer shooter_srv = n.advertiseService("/seabee3/ShooterAction", ShooterActionCB);
		
	while(ros::ok())
	{
	  seabee3_driver_base::Pressure intlPressureMsg;
	  seabee3_driver_base::Pressure extlPressureMsg;
	  seabee3_driver_base::KillSwitch killSwitchMsg;
	  
	  mDriver->readPressure(intlPressureMsg.Value, extlPressureMsg.Value);
	  mDriver->readKillSwitch(killSwitchMsg.Value);
	  
	  intl_pressure_pub.publish(intlPressureMsg);
	  extl_pressure_pub.publish(extlPressureMsg);
	  kill_switch_pub.publish(killSwitchMsg);
	  
	  ros::spinOnce();
	  ros::Rate(20).sleep();
	}
	ros::spin();
	return 0;
}
