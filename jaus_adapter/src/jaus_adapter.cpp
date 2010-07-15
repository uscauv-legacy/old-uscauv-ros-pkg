/*******************************************************************************
 *
 *      jaus_adapter 
 * 
 *      Copyright (c) 2010, Johnny 'Pimp Masta' O'Hollaren, more@cow.bell
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
 *      * Neither the name of the USC Robotics Team nor the names of its
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



// TODO
// Does each msg_id need to be unique? 
// try with both full and not full msg structs

#include "JuniorAPI.h"

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>

#include <string>
#include <sstream>
#include <math.h>
#include <stdio.h>

#include "convert.h" // shoddy int to string conversion
#include <sys/time.h>
#include <time.h>

unsigned int id = 			0x00870101; // 0x87 = 135, my ID 
unsigned int destination = 	0x005A0101; // COP: subsystem ID is 90 - 0x005A hex

using namespace std;

// Define the message structure.  We have to tell
// the compiler to pack on 1-byte boundaries to
// make sure it doesn't insert padding that would
// be non-compliant to AS6009.
#pragma pack(1)

//----------------------------------------------------------------
// DEFINE MESSAGES TO BE PASSED
//----------------------------------------------------------------


// Report Velocity
typedef struct {
	unsigned short 	msg_id;
	unsigned short 	pv;
	unsigned int   	vel_x;
//	unsigned int   	vel_y;
//	unsigned int   	vel_z;
//	unsigned int   	vel_rms;
//	unsigned short  roll_rate;
//	unsigned short  pitch_rate;
//	unsigned short  rate_rms;
//	unsigned short  yaw_rate;
//	unsigned int   	time_stamp;
} REPORT_VELOCITY_STATE_MSG;

// Report Pose
typedef struct {
	unsigned short 	msg_id;
	unsigned short 	pv;
	unsigned int   	pos_x;
//	unsigned int   	pos_y;
//	unsigned int   	pos_z;
//	unsigned int   	pos_rms;
//	unsigned short 	roll;
//	unsigned short 	pitch;
	unsigned short 	yaw;
//	unsigned short  altitude_rms;
//	unsigned int   	time_stamp;*/
} REPORT_LOCAL_POSE_MSG;

//----------------------------------------------------------------
// Define some helper functions
//----------------------------------------------------------------

// create 16 bit scale int
unsigned short scaleToUInt16(float val, float low, float high)
{
	float int_range = pow(2, 16) - 1;
	float scale_factor = (high-low)/int_range;
	return (unsigned short)((val-low)/scale_factor);
}

// unscale 16 bit scaled int
float unscaleFromUInt16(unsigned short val, float low, float high)
{
	float int_range = pow(2, 16) - 1;
	float scale_factor = (high-low)/int_range;
	return scale_factor * ((float) val) + low;
}

// create 32 bit scaled int
unsigned int scaleToUInt32(double val, double low, double high)
{
	double int_range = pow(2, 32) - 1;
	double scale_factor = (high-low)/int_range;
	return (unsigned int)((val-low)/scale_factor);
}

// unscale 32 bit scaled int
double unscaleFromUInt32(unsigned int val, double low, double high)
{
	double int_range = pow(2, 32) - 1;
	double scale_factor = (high-low)/int_range;
	return scale_factor * ((double) val) + low;
}

//----------------------------------------------------------------
// Global Variables because I suck at programming
//----------------------------------------------------------------

// variables for receiving messages
char buffer[1000];          	// Allocate the space for a received message
unsigned int size = 1000;      	// Initialize size
unsigned int source;            // This value is returned as an ID from the
								// sender of the message
	
//----------------------------------------------------------------
// Begin Main
//----------------------------------------------------------------
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
		ROS_FATAL("The jaus_adapter node must be run from it's own bin directory!!!");
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

	//----------------------------------------------------------------
	// TASK 2 - CAPABILITIES DISCOVERY
	//----------------------------------------------------------------

	// This task has two parts. First, the COP will send a "Query Services"
	// message. After we receive that, we will respond with a "Report Services"
	// message.

	// First we will wait for the Query Services message. We will keep looping
	// until we find it.
	// as5710 Page 50
/*
	// check for Query Services message received
	bool query_services_received = false; 
	while ( query_services_received == false)
	{
		if (JrReceive( handle, &source, &size, buffer) == NoMessages)
			cout << "\nNo Message Received Yet...\n\n";
		else 
		{
			cout << "\nReceived " << size << " bytes from " << source << "\n\n";
			cout << "\n***Received Message***\n" << buffer << "\n\n\n";
			query_services_received = true;
		}
		ros::Duration(1).sleep();
	}

	// Now that the Query Services message has been received, we will send
	// back the "Report Services" message.
	// as5710 Page 56

	REPORT_SERVICES_MSG msg_serv;
	msg_serv.msg_id = 
	msg_serv.node_id =
	msg_serv.component_id =
	msg_serv.instance_id =
	msg_serv.uri =
	msg_serv.version_mjr =
	msg_serv.version_mnr =

	bool report_services_sent = false;
	while ( report_services_sent == false)
	{
		if (JrSend( handle, destination, sizeof(msg_serv), (char*)&msg_serv) != Ok)
		{
			cout << "\nFailing to send Report Services Message...\n\n";
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

	// Receive Query Control
	// as5710 page 47

	// Send Report Control
	// as5710 Page 52

	// Recieve Request Control
	// as5710 Page 41

	// Send Confirm Control
	// as5710 Page 41
	
	// Receive Query Status
	// as5710 Page 46

	// Send Report Status
	// as5710 Page 51

	// State machine for resume standbuy shutdown stuff 
	
	//----------------------------------------------------------------
	// TASK 4 - VELOCITY STATE REPORT
	//----------------------------------------------------------------
	// COP sends: Query Velocity State, return: Report Velocity State
	//     Velocity X, Raw Rate, & Time Stamp [320 Decimal, 0104h]
	// Units are meters per second
/*
	// Receive Query Velocity
	// Page 66 in as6009
	bool query_velocity_received = false; 
	while ( query_velocity_received == false)
	{
		if (JrReceive( handle, &source, &size, buffer) == NoMessages)
			cout << "\nNo QUERY VELOCITY Message Received Yet...\n\n";
		else 
		{
			cout << "\nReceived " << size << " bytes of data from " << source << "\n\n";
			cout << "\n***Received Message***\n" << buffer << "\n\n\n";
			query_velocity_received = true;
		}
		ros::Duration(1).sleep();
	}
*/

	// Now send Report Velocity State
	// pv = 1 for competition
	// Page 72 in as6009, ID 4402
	REPORT_VELOCITY_STATE_MSG msg_vel;

	msg_vel.msg_id 		= 0x4402;
	msg_vel.pv 			= 1; 
	msg_vel.vel_x 		= scaleToUInt32(200, -327.68, 327.67);
//	msg_vel.vel_y 		= scaleToUInt32(0, -327.68, 327.67);
//	msg_vel.vel_z 		= scaleToUInt32(0, -327.68, 327.67);
//	msg_vel.vel_rms 	= scaleToUInt32(0, 0, 100);
//	msg_vel.roll_rate 	= scaleToUInt16(0, -32.768, 32.767);
//	msg_vel.pitch_rate 	= scaleToUInt16(0, -32.768, 32.767);
//	msg_vel.yaw_rate 	= scaleToUInt16(0, -32.768, 32.767);
//	msg_vel.rate_rms 	= scaleToUInt16(0, 0, 3.14);
//	msg_vel.time_stamp 	= scaleToUInt32(0);

	// Now we send the message to the COP using Junior.  Recall
	// that the COP subsystem id is decimal 90 (0x005A hex)
	if (JrSend(handle, destination, sizeof(msg_vel), (char*)&msg_vel) != Ok)
		cout << "\n\t *** Unable to Send Message *** \n\n";
	else 
	{
		cout << "\n *** Successfully Sent REPORT VELOCITY STATE Message  ***\n";
		cout << "Velocity X = " << msg_vel.vel_x << "\n\n";
	}
	ros::Duration(3).sleep();

	//----------------------------------------------------------------
	// TASK 5 - POSITION AND ORIENTATION REPORT
	//----------------------------------------------------------------
	// COP sends: Query Local Pose, return: Report Local Pose
	//     Yaw & Time Stamp [320 Decimal, 0140h]

	// Receive Query Local Pose
	// Page 66 in as6009

	// Now Send Report Local Pose
	// Page 53 in as6009 
	REPORT_LOCAL_POSE_MSG msg_pose; // create message for sending yaw

	msg_pose.msg_id 		= 0x4403;
	msg_pose.pv 			= 65; 
	msg_pose.pos_x 			= scaleToUInt32(4.0, -100000, 100000);
//	msg_pose.pos_y 			= 0;
//	msg_pose.pos_z 			= 0;
//	msg_pose.pos_rms 		= 0;
//	msg_pose.roll 			= 0;
//	msg_pose.pitch 			= 0;
	msg_pose.yaw 			= scaleToUInt16(2.0, -3.14, 3.14); 
//	msg_pose.altitude_rms 	= 0;
//	msg_pose.time_stamp 	= 0;  

	// Now we send the message to the COP using Junior.  Recall
	// that the COP subsystem id is decimal 90 (0x005A hex)
	if (JrSend(handle, destination, sizeof(msg_pose), (char*)&msg_pose) != Ok)
		cout << "\n\t *** Unable to Send Message *** \n\n";
	else 
	{
		cout << "\n *** Successfully Sent REPORT LOCAL POSE Message  ***\n";
		cout << "Yaw = " << msg_pose.yaw << "\n\n";
	}
	ros::Duration(3).sleep();

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


/*
unsigned int GetTimeOfDay()
{
	struct timeval tv;
	struct tm *timeinfo;
	time_t rawtime;
	unsigned int tstamp = 0;
	
	gettimeofday(&tv, NULL);
	time(&rawtime);
	timeinfo = gmtime ( &rawtime );
	
	unsigned int mMilliseconds = (unsigned int)(tv.tv_usec/1000.0);
	int mDay = timeinfo->tm_mday;
	int mHour = timeinfo->tm_hour;
	int mMinute = timeinfo->tm_min;
	int mSecond = timeinfo->tm_sec;
	
	tstamp |= (unsigned int)(mDay)    << 27;
	tstamp |= (unsigned int)(mHour)   << 22;
	tstamp |= (unsigned int)(mMinute) << 16;
	tstamp |= (unsigned int)(mSecond) << 10;
	tstamp |= mMilliseconds;
	//cout << "Time Stamp: " << mDay << ":" << mHour << ":" << mMinute;
    //cout << ":" << mSecond << ":" << mMilliseconds << endl;
	return tstamp ;
}
*/
