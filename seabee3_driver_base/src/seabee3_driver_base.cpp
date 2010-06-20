#include <ros/ros.h>
#include "seabee3_driver.h"
#include "seabee3_driver_base/Pressure.h"

seabee3_driver * mDriver;

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

int main (int argc, char** argv)
{
	ros::init(argc, argv, "seabee3_driver_base");
	ros::NodeHandle n;
	
	ROS_INFO("constructing new driver instance");
	mDriver = new seabee3_driver;
	
	ROS_INFO("subscribing to MotorCntl");
	ros::Subscriber motor_cntl_sub = n.subscribe("MotorCntl", 100, motorCntlCallback);
	ros::Publisher intl_pressure_pub = n.advertise<seabee3_driver_base::Pressure>("IntlPressure", 100);
	ros::Publisher extl_pressure_pub = n.advertise<seabee3_driver_base::Pressure>("ExtlPressure", 100);
		
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
