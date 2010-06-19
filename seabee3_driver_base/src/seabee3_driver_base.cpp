#include <ros/ros.h>
#include "seabee3_driver.h"

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
	
	mDriver = new seabee3_driver;
	
	ros::Subscriber motor_cntl_sub = n.subscribe("MotorCntl", 100, motorCntlCallback);
	
	/*while(ros::ok())
	{
		mDriver->step()
		ros::spinOnce();
		ros::Rate(20).sleep();
	}*/
	ros::spin();
	return 0;
}
