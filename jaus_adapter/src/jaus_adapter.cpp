/**************************************************************************
 *
 * jaus_adapter 
 *
 * Copyright (c) 2010, Johnny 'Pimp Masta' O'Hollaren, more@cow.bell
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 * 
 * * Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above
 *   copyright notice, this list of conditions and the following disclaimer
 *   in the documentation and/or other materials provided with the
 *   distribution.
 * * Neither the name of the USC Robotics Team nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 **************************************************************************/

#include "JuniorAPI.h"
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <vector>
#include <string>
#include <sstream>
#include <math.h>
#include <stdio.h>
#include "convert.h" 
#include <sys/time.h>
#include <time.h>

using namespace std;

//################################################################
//################################################################

unsigned int id = 					0x00870101; // 0x87 = 135, my ID 
unsigned int destination = 	0x005A0101; // COP: subsystem ID is 90 - 0x005A hex
#define MAX_BUFFER_SIZE 1000

//################################################################
//################################################################

// variables for receiving messages
char buffer[MAX_BUFFER_SIZE];          	// Allocate the space for a received message
unsigned int size = MAX_BUFFER_SIZE;   	// Initialize size
unsigned int source;            				// This value is returned as an ID from the
int *priority; 
int *flags; 
unsigned short *msg_id; 

// unique identifier for transaction between sender and receiver
long int handle;						

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
} REPORT_VELOCITY_STATE_MSG;
REPORT_VELOCITY_STATE_MSG msg_vel;

// Report Pose
typedef struct {
	unsigned short 	msg_id;
	unsigned short 	pv;
	unsigned short 	yaw;
} REPORT_LOCAL_POSE_MSG;
REPORT_LOCAL_POSE_MSG msg_pose; 

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

	// We have to call JrConnect before we send any messages.
	// JrConnect requires us to pass in the JAUS ID for our
	// component as well as the configuration file.  We 
	// get a handle back that's used for subsequent calls.
	// I'm using a subsystem ID (in hex), 
	// and node id 1, component id 1.

	//------------------------------------------------------------------
	// INITIATE HANDSHAKE
	//------------------------------------------------------------------
	cout << "\n\n------------------------------------------------";
	cout << "\n\tBEGINNING CONNECTION";
	cout << "\n------------------------------------------------\n\n";

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

	//------------------------------------------------------------------
	// START INTERACTION
	//------------------------------------------------------------------

	// For the rest of the tasks, the COP initatives everything
	while(ros::ok())
	{
		//------------------------------------------------------------------
		// LOOP AND RECEIVE MESSAGES
		//------------------------------------------------------------------
		if (JrReceive( handle, &source, &size, buffer, 
					priority, flags, msg_id) == NoMessages)
		{   
			cout << "\nNo Message Received Yet...\n\n";
			ros::Duration(1).sleep();
		}

		// if we get a message, then do something depending on what message it was
		else 
		{
			// print info about the message
			cout << "\n\nMSG_ID = " << msg_id;
			cout << "\nReceived " << size << " bytes from " << source << "\n";

			// then do something depending on what the message ID was
			switch( (int) msg_id )
			{
				//--------------------------------------------------------------
				// REPORT LOCAL POSE
				//--------------------------------------------------------------
				case 0x2403: // msg_id for Query Local Pose, [as6009, 66]
					// send Report Local Pose, 0x4403 [as6009, 72]
					msg_pose.msg_id 	= 0x4403;
					msg_pose.pv 			= 65; 
					msg_pose.yaw 			= scaleToUInt16(2.0, -3.14, 3.14); 

					// Now we send the message to the COP using Junior.  Recall
					// that the COP subsystem id is decimal 90 (0x005A hex)
					if (JrSend(handle, destination, sizeof(msg_pose), (char*)&msg_pose) != Ok)
						cout << "\n\t *** Unable to Send Message *** \n\n";
					else 
					{
						cout << "\n *** REPORT LOCAL POSE sent ***\n";
						cout << "Yaw = " << msg_pose.yaw << "\n\n";
					}

					//--------------------------------------------------------------
					// REPORT VELOCITY STATE
					//--------------------------------------------------------------
				case 0x2404: // msg_id for Query Velocity State
					// Now send Report Velocity State, 0x4402 [as6009, 72]
					msg_vel.msg_id 		= 0x4402;
					msg_vel.pv 				= 1; // 1 for competition 
					msg_vel.vel_x 		= scaleToUInt32(200, -327.68, 327.67);

					// Now we send the message to the COP using Junior.  Recall
					// that the COP subsystem id is decimal 90 (0x005A hex)
					if (JrSend(handle, destination, sizeof(msg_vel), (char*)&msg_vel) != Ok)
						cout << "\n\t *** Unable to Send Message *** \n\n";
					else 
					{
						cout << "\n *** REPORT VELOCITY STATE sent ***\n";
						cout << "Velocity X = " << msg_vel.vel_x << "\n\n";
					}

					//--------------------------------------------------------------
					// SYSTEM MANAGEMENT (next few case statements)
					//--------------------------------------------------------------
				case 0x200D: // msg_id for Query Control [as5710, 47] 
					// send Report Control, 0x400D [as5710, 52]
					continue;

				case 0x000D: // msg_id for Request Control [as5710, 41]
					// send Confirm Control, 0x000F [as5710, 41] 
					continue;

				case 0x2002: // msg_id for Query Status [as5710, 46]
					// send Report Status, 0x4002 [as5710, 51]
					// 0 INIT
					// 1 READY
					// 2 STANDBY
					// 3 SHUTDOWN
					// 4 FAILURE
					// 5 EMERGENCY
					continue;

					//--------------------------------------------------------------
					// CAPABILITIES 
					//--------------------------------------------------------------
				case 0x2B03: // msg_id for Query Services [as5710, 50]
					// send Report Services, 0x4B03 [as5710, 56]
					continue;


			}
		}
	}
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
