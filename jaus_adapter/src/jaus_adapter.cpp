#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>

#include <string>

#include <seabee3_driver/SetDesiredRPY.h>
#include <seabee3_driver/SetDesiredXYZ.h>
#include <xsens_node/IMUData.h>
#include <waypoint_controller/SetDesiredPose.h>
#include <waypoint_controller/ReleasePose.h>
#include <seabee3_driver_base/Pressure.h>

int jaus_port;
std::string cop_ip;

struct JausID
{
	int ssid;
	int node;
	int comp;
};

JausID cop, self;

int main( int argc, char* argv[] )
{
	ros::init(argc, argv, "jaus_adapter");
	ros::NodeHandle n("~");
	
	n.param("port", jaus_port, 3794);
	
	n.param("cop/ip", cop_ip, std::string("192.168.1.42") );
	
	n.param("cop/id/ssid", cop.ssid, 42 );
	n.param("cop/id/node", cop.node, 1 );
	n.param("cop/id/comp", cop.comp, 1 );
	
	n.param("self/id/ssid", self.ssid, 0 )
	n.param("self/id/node", self.node, 1 )
	n.param("self/id/comp", self.comp, 1 )
	
	ros::spin();
	return 1;
}
