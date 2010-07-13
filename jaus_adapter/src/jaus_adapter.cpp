#include "JuniorAPI.h"

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>

#include <string>
#include <sstream>
#include <math.h>
#include <stdio.h>
#include "convert.h" // int to string conversion i threw together

using namespace std;

// Define the message structure.  We have to tell
// the compiler to pack on 1-byte boundaries to
// make sure it doesn't insert padding that would
// be non-compliant to AS6009.
#pragma pack(1)
typedef struct {
  unsigned short msg_id;
  unsigned short pv;
  unsigned int   X;
  unsigned int   Y;
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

	char cwdBuffer[256];
	std::string cwd(getcwd(cwdBuffer, 256));
	std::cout << cwd << std::endl;
	if(cwd.substr(cwd.size()-3, cwd.size()) != "bin") 
	{
		ROS_FATAL("The jaus_adapter node _must_ be run from within it's own bin directory.");
	}

	cout << "\n----------- BEGINNING CONNECTION ---------------";
	cout << "\n------------------------------------------------\n\n";

	// CONNECT
	// We have to call JrConnect before we send any messages.
    // JrConnect requires us to pass in the JAUS ID for our
    // component as well as the configuration file.  We 
    // get a handle back that's used for subsequent calls.
    // I'm using a subsystem ID of 155 decimal (0x009B hex) 
    // and node id 1, component id 1.
	long int handle;
	int addr = 0x00000101;
	addr |= 144 << 4;
	addr &= 0x01111111;

//0x00850101
    if (JrConnect(addr, "jr_config.xml", &handle) != Ok)
    {
        cout << "\nFailed to connect to Junior.\n\n";
        return 0;
    }
    else 
		cout << "\nSuccessfully connected to Junior...\n\n";

	//while(ros::ok())
	{
    // Create the message and check it's size.  We want to make
    // sure the compiler didn't mess with the structure.
    REPORT_LOCAL_POSE_MSG msg;
    if (sizeof(msg) != 12) 
		cout << "\nPacking error!\n\n";

    // Populate the message.  The message id is fixed, but the
    // X and Y data are bogus.  The PV is set to indicate that
    // the first 2 optional fields are present.
    msg.msg_id = 0x4403;
    msg.pv = 3;
    msg.X = scaleToUInt32(999.555555, -100000, 100000);
    msg.Y = scaleToUInt32(-888.3, -100000, 100000);

		// Now we send the message to the COP using Junior.  Recall
		// that the COP subsystem id is decimal 90 (0x005A hex)
		if (JrSend(handle, 0x005A0101, sizeof(msg), (char*)&msg) != Ok)
			cout << "\nUnable to send message.\n\n";
		else 
			cout << "\nSent message to the COP\n\n";

		ros::spinOnce();
		ros::Rate(1).sleep();
	}

  // Clean-up
	cout << "\n\n------------------Disconnecting--------------\n\n";
	JrDisconnect(handle);
	cout << "\n\n\n\n\n\n\n------------------Disconnected!--------------\n\n\n\n\n";

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
