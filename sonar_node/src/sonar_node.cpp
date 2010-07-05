#include <ros/ros.h>
#include <sonar_node/SonarScanArray.h>

int main( int argc, char *argv[])
{
	ros::init(argc, argv, "sonar_node");
	ros::NodeHandle n;
	
	ros::spin();
	return 1;
}
