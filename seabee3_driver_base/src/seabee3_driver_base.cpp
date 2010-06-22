#include <ros/ros.h>
#include <seabee3_beestem/BeeStem3_driver.h>
#include <seabee3_driver_base/Pressure.h>
#include <seabee3_driver_base/MotorCntl.h>
#include <seabee3_driver_base/Dropper1Action.h>
#include <seabee3_driver_base/Dropper2Action.h>
#include <seabee3_driver_base/ShooterAction.h>

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

bool Dropper1ActionCB(seabee3_driver_base::Dropper1Action::Request &req, seabee3_driver_base::Dropper1Action::Response &res)
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

bool Dropper2ActionCB(seabee3_driver_base::Dropper2Action::Request &req, seabee3_driver_base::Dropper2Action::Response &res)
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

bool ShooterActionCB(seabee3_driver_base::ShooterAction::Request &req, seabee3_driver_base::ShooterAction::Response &res)
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
	ros::NodeHandle n;
	
	n.param("seabee3_driver_base/usbIndex", usbIndex, 0);
	
	ROS_INFO("constructing new driver instance");
	mDriver = new BeeStem3Driver(usbIndex);
	
	ROS_INFO("subscribing to MotorCntl");
	ros::Subscriber motor_cntl_sub = n.subscribe("seabee3/MotorCntl", 100, motorCntlCallback);
	
	ros::Publisher intl_pressure_pub = n.advertise<seabee3_driver_base::Pressure>("seabee3/IntlPressure", 100);
	ros::Publisher extl_pressure_pub = n.advertise<seabee3_driver_base::Pressure>("seabee3/ExtlPressure", 100);
	
	ros::ServiceServer dropper1action_srv = n.advertiseService("seabee3/Dropper1Action", Dropper1ActionCB);
	ros::ServiceServer dropper2action_srv = n.advertiseService("seabee3/Dropper2Action", Dropper2ActionCB);
	ros::ServiceServer shooter_srv = n.advertiseService("seabee3/ShooterAction", ShooterActionCB);
		
	while(ros::ok())
	{
		seabee3_driver_base::Pressure intlPressureMsg;
		seabee3_driver_base::Pressure extlPressureMsg;
		
		mDriver->readPressure(intlPressureMsg.Value, extlPressureMsg.Value);
		
		intl_pressure_pub.publish(intlPressureMsg);
		extl_pressure_pub.publish(extlPressureMsg);
		
		ros::spinOnce();
		ros::Rate(20).sleep();
	}
	ros::spin();
	return 0;
}
