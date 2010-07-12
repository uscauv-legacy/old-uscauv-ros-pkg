#include "JuniorAPI.h"

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>

#include <string>
#include <sstream>
#include "convert.h" // int to string conversion

using namespace std;

// id holding struct
struct JausID
{
	unsigned int ssid;
	unsigned int node;
	unsigned int component;
	string jaus_id;
};

// make one instance for the COP (Common Operating Picture)
// server, and one for SeaBee
JausID cop, self;

int main( int argc, char* argv[] )
{
	ros::init(argc, argv, "jaus_adapter");
	ros::NodeHandle n("~");

	// all jaus traffic is on port 3794
	int jaus_port;
	jaus_port = 3794;
	
	// they gave us this as the IP for the COP
	string cop_ip;
	cop_ip = "192.16.1.42";

	// set COP's id struct
	cop.ssid = 42;
	cop.node = 1;
	cop.component = 1;
	cop.jaus_id = stringify(cop.ssid) + ":" + stringify(cop.node) 
								+ ":" + stringify(cop.component);

	// set SeaBee's id struct
	self.ssid = 100;
	self.node = 1;
	self.component = 1;
	self.jaus_id = stringify(self.ssid) + ":" + stringify(self.node) 
								+ ":" + stringify(self.component);

	// print out id information for COP and SeaBee
	cout << "\n----------- JAUS IDs ---------------";
	cout << "\nSeaBee ID: \t" << self.jaus_id;
	cout << "\nCOP ID: \t" << cop.jaus_id;
	cout << "\n------------------------------------\n\n";

	// CONNECT
	long handle;
	int return_code;
	return_code = JrConnect( self.ssid, "jr_config.xml", &handle );

	// TASK 1 - DISCOVERY
	// COP sends: Query Identification, return: Report Identification

	// TASK 2 - CAPABILITIES DISCOVERY
	// COP sends: Query Services, return: Report Services 

	// TASK 3 - SYSTEM MANAGEMENT
	// COP sends: Query Control, return: Report Control
	// COP sends: Request Control, return: Confirm Control
	// COP sends: Query Status, return: Report Status
	// COP sends: Resume
	// COP sends: Standby
	// COP sends: Shutdown

	// TASK 4 - VELOCITY STATE REPORT
	// COP sends: Query Velocity State, return: Report Velocity State
	//     Velocity X, Raw Rate, & Time Stamp [320 Decimal, 0104h]

	// TASK 5 - POSITION AND ORIENTATION REPORT
	// COP sends: Set Local Pose
	//     X, Y, & Yaw [67 Decimal, 0043h]
	// COP sends: Query Local Pose, return: Report Local Pose
	//     Yaw & Time Stamp [320 Decimal, 0140h]


	
	
	ros::spin();
	return 1;
}
