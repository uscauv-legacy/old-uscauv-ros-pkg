#include "JuniorAPI.h"

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>

#include <string>
#include <sstream>
#include <math.h>
#include <stdio.h>
#include "convert.h" // shoddy int to string conversion

unsigned int id = 			0x008E0101; // hex 142 
unsigned int destination = 	0x005A0101; // COP: subsystem ID is 90 - 0x005A hex

using namespace std;

// Define the message structure.  We have to tell
// the compiler to pack on 1-byte boundaries to
// make sure it doesn't insert padding that would
// be non-compliant to AS6009.
#pragma pack(1)
typedef struct {
	unsigned short msg_id;
	unsigned short pv;
	unsigned int   yaw;
	unsigned int   time_stamp;
} REPORT_LOCAL_POSE_MSG;

// Define some helper functions to convert to/from
// scaled integers.
unsigned int scaleToUInt32(double val, double low, double high)
{
	double int_range = pow(2, 32) - 1;
	double scale_factor = (high-low)/int_range;
	return (unsigned int)((val-low)/scale_factor);
}
double unscaleFromUInt32(unsigned int val, double low, double high)
{
	double int_range = pow(2, 32) - 1;
	double scale_factor = (high-low)/int_range;
	return scale_factor * ((double) val) + low;
}

int main( int argc, char* argv[] )
{
	ros::init(argc, argv, "jaus_adapter");
	ros::NodeHandle n("~");

	// make sure we are running this from its own bin directory
	char cwdBuffer[256];
	std::string cwd(getcwd(cwdBuffer, 256));
	std::cout << cwd << std::endl;
	if(cwd.substr(cwd.size()-3, cwd.size()) != "bin") 
	{
		ROS_FATAL("STOP!!! The jaus_adapter node _must_ be run from within it's own bin directory!!!");
	}

	//----------------------------------------------------------------
	// TASK 1 - CONNECT
	//----------------------------------------------------------------

	// We have to call JrConnect before we send any messages.
	// JrConnect requires us to pass in the JAUS ID for our
	// component as well as the configuration file.  We 
	// get a handle back that's used for subsequent calls.
	// I'm using a subsystem ID (in hex), 
	// and node id 1, component id 1.

	cout << "\n\n------------------------------------------------";
	cout << "\n\tBEGINNING CONNECTION";
	cout << "\n------------------------------------------------\n\n";

	long int handle;

	bool connection_established = false;
	while( connection_established == false)
	{
		if (JrConnect( id, "jr_config.xml", &handle) != Ok)
		{
			cout << "\nFailed to connect to Junior.\n\n";
			return 0;
		}
		else 
		{
			cout << "\n\n------------------------------------------------";
			cout << "\n\tSuccessfully Connected\n";
			cout << "\tHandle: " << handle << "\n";
			cout << "------------------------------------------------\n\n";
			connection_established = true;
		}
		ros::Duration(5).sleep();
	}

	ros::spinOnce();

	//----------------------------------------------------------------
	// TASK 2 - CAPABILITIES DISCOVERY
	//----------------------------------------------------------------

	// This task has two parts. First, the COP will send a "Query Services"
	// message. After we receive that, we will respond with a "Report Services"
	// message.

	// First we will wait for the Query Services message. We will keep looping
	// until we find it.

	char buffer[1000];          	// Allocate the space
	unsigned int size = 1000;      	// Initialize size
	unsigned int source;            // This value is returned as an ID from the
									// source that sent the message

/*
	// check for Query Services message received
	bool query_services_received = false; 
	while ( query_services_received == false)
	{
		if (JrReceive( handle, &source, &size, buffer) == NoMessages)
			cout << "\nNo Message Received Yet...\n\n";
		else 
		{
			cout << "\nReceived " << size << " bytes of data from " << source << "\n\n";
			cout << "\n***Received Message***\n" << buffer << "\n\n\n";
			query_services_received = true;
		}
		ros::Duration(1).sleep();
	}
*/
/*
	// Now that the Query Services message has been received, we will send
	// back the "Report Services" message.
	bool report_services_sent = false;
	while ( report_services_sent == false)
	{
		if (JrSend( handle, destination, "jr_config.xml", &handle) != Ok)
		{
			cout << "\nFailing to send report Services Message...\n\n";
		}
		else 
		{
			cout << "\nSuccessfully Sent Report Services Message\n\n";
			report_services_sent = true;
		}
		ros::Duration(1).sleep();
	}
*/








	//----------------------------------------------------------------
	// TASK 3 - SYSTEM MANAGEMENT
	//----------------------------------------------------------------
	// COP sends: Query Control, return: Report Control
	// COP sends: Request Control, return: Confirm Control
	// COP sends: Query Status, return: Report Status
	// COP sends: Resume
	// COP sends: Standby
	// COP sends: Shutdown

	//----------------------------------------------------------------
	// TASK 4 - VELOCITY STATE REPORT
	//----------------------------------------------------------------
	// COP sends: Query Velocity State, return: Report Velocity State
	//     Velocity X, Raw Rate, & Time Stamp [320 Decimal, 0104h]

	//----------------------------------------------------------------
	// TASK 5 - POSITION AND ORIENTATION REPORT
	//----------------------------------------------------------------
	// COP sends: Query Local Pose, return: Report Local Pose
	//     Yaw & Time Stamp [320 Decimal, 0140h]

	REPORT_LOCAL_POSE_MSG msg; // create message for sending yaw

	// Populate the message.  The message id is fixed, but the
	// X and Y data are bogus.  The PV is set to indicate that
	// the first 2 optional fields are present.
	msg.msg_id = 0x4403;
	msg.pv = 320;
	msg.yaw = scaleToUInt32(10, -100000, 100000);
	msg.time_stamp = scaleToUInt32(0xFFFF0000, -100000, 100000); 

	// Now we send the message to the COP using Junior.  Recall
	// that the COP subsystem id is decimal 90 (0x005A hex)
	// page 53 in as6009
	if (JrSend(handle, destination, sizeof(msg), (char*)&msg) != Ok)
		cout << "\n\t *** Unable to Send Message *** \n\n";
	else 
		cout << "\n *** Successfully Sent Yaw and Timestamp to COP ***\n\n";

	//----------------------------------------------------------------
	// Clean-up
	//----------------------------------------------------------------
	JrDisconnect(handle);
	cout << "\n\n------------------------------------------------";
	cout << "\n\tDisconnected\n";
	cout << "------------------------------------------------\n\n";

	ros::spinOnce();
	return 0;
}
