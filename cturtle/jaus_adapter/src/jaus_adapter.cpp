/**************************************************************************
 *
 * jaus_adapter 
 *
 * Copyright (c) 2010, John O'Hollaren, ohollare@usc.edu
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

// ask:
// 1. Strings in report services, report id?
// 2. 

//################################################################
//
// INFORMATION
// 
// Wiki Link: http://code.google.com/p/seabee3-ros-pkg/wiki/JAUS_Interface
// 
// Dependencies (already configured):
// 		JrMiddleWare
//			1. jr_config.xml: XML config file in bin directory
//			2. JuniorRTE: Executable in bin directory
//
// To Make It Work, change variables below where it says
// CHANGE THESE VARIABLES TO MAKE IT WORK: 
// 			Change "id" to your ID:
// 				id = 0x00<last oclet of ip in hex>0101
//			Change "destination" to Correct Value
//        -If Using JVT, comment top COP 'destination' and
//				 uncomment one labeled JVT
//				-If using the real COP, comment JVT 'destination'
//				 and uncomment the one labeled COP.
//      If using a destination other than those two, you will
//      need to add it to the jr_config.xml file address book
//      in /bin
//
//################################################################

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
#include <xsens_node/Imu.h>
#include <boost/thread/thread.hpp> 
#include <boost/thread/mutex.hpp>

using namespace std;

//################################################################
// CHANGE THESE VARIABLES TO MAKE IT WORK
//################################################################

unsigned int id = 					0x00820101; // 0x82 = 130, my ID 

// uncomment below destination for real COP Testing
unsigned int destination = 	0x002A0101;	 
// 0x002A0101 is ID for COP station with IP: 192.168.1.42:3794

// uncomment below destination for JVT TESTING 
// unsigned int destination = 0x005A0101;
// 0x005A0101 is ID for JVT server with IP: 24.42.140.203:3794

//################################################################
// LEAVE THESE ALONE
//################################################################

// Whole bunch of global variables because I'm a l33t programmer 

// Define the message structure.  We have to tell
// the compiler to pack on 1-byte boundaries to
// make sure it doesn't insert padding that would
// be non-compliant to AS6009.
//#pragma pack(1)

#define MAX_BUFFER_SIZE 		1000				// size of data sent back 

// variables for receiving messages
char buffer[MAX_BUFFER_SIZE];          	// Allocate space for message
unsigned int size = MAX_BUFFER_SIZE;   	// Initialize size
unsigned int source;            				// Returned ID value 
int priority; 
int flags; 
unsigned short msg_id; 

// hold incoming data
char next_byte;
unsigned short next_2_bytes;
unsigned int next_4_bytes;

// unique identifier for transaction between sender and receiver
long int handle;						

// Jr Outputs
int jr_send_rtn;
int jr_receive_rtn;
int jr_connect_rtn;

// Counters for super realistic incremental simulation of data
float 	sim_short 	= 0; // for simulating yaw
double 	sim_reg 		= 0; // for simulating pos_x

unsigned char state = 0x02;

//################################################################
// DEFINE MESSAGES TO BE PASSED
//################################################################

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

// Report Control
typedef struct {
	unsigned short	msg_id;			 	
	unsigned short  ss_id;				// value_range (0, 65535)
	unsigned short  id_stuff;
	char		auth_code;    // value_range (0, 254)
} REPORT_CONTROL_MSG;
REPORT_CONTROL_MSG msg_ctrl;

// Confirm Control
typedef struct {
	unsigned short 	msg_id;
	unsigned char  	response_code; // value_enum = 0,1,2
} CONFIRM_CONTROL_MSG;
CONFIRM_CONTROL_MSG msg_cfrm;

// Report Status
typedef struct {
	unsigned short 	msg_id;
	unsigned char 	 	status;
	unsigned int		reserved;
}	REPORT_STATUS_MSG;
REPORT_STATUS_MSG msg_status;

// Query Identification
typedef struct {
	unsigned short 	msg_id;
	unsigned int		query_type;
} QUERY_ID_MSG;
QUERY_ID_MSG msg_id_init;

// Report Identification
typedef struct {
	unsigned short  	msg_id;
	unsigned char   	query_type;
	unsigned short  	type;
	unsigned char			byte;
	unsigned char			id[6];					
} REPORT_ID_MSG;
REPORT_ID_MSG msg_report_id;

// Report Services
// Serialize all these inherited messages together
typedef struct {
	unsigned short	msg_id;
	unsigned char		node_list;
	unsigned char 	node_id;
	unsigned char 	component_list;
	unsigned char 	component_id;
	unsigned char 	instance_id;
	unsigned char 	service_list;
	unsigned char		dummy;
	unsigned char		uri[5];
	unsigned char		version_major;
	unsigned char		version_minor;
} REPORT_SERVICES_MSG;
REPORT_SERVICES_MSG msg_service;

//################################################################
// Define some helper functions
//################################################################

//################################################################
//################################################################
//################################################################
//################################################################
// EDWARD!!!!!!!!!!!!!!!!!
// FILL IN THIS FUNCTION BELOW (get_yaw)
//################################################################
//################################################################
//################################################################
//################################################################

float mYaw = 0.0;
boost::mutex imu_mutex;

void imuCallback(const xsens_node::ImuConstPtr & msg)
{
  mYaw = msg->ori.z;
}

unsigned short get_yaw()
{
  return (unsigned short)mYaw;
}

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

//################################################################
// Begin Main
//################################################################
int main( int argc, char* argv[] )
{
	ros::init(argc, argv, "jaus_adapter");
	ros::NodeHandle n("~");

	ros::Subscriber imu_sub = n.subscribe("/xsens/data_calibrated", 1, imuCallback);

	// make sure we are running this from its own bin directory
	char cwdBuffer[256];
	std::string cwd(getcwd(cwdBuffer, 256));
	std::cout << cwd << std::endl;
	if(cwd.substr(cwd.size()-3, cwd.size()) != "bin") 
	{
		ROS_FATAL("Jaus_adapter must be run from it's own bin directory!!!");
	}

	// We have to call JrConnect before we send any messages.
	// JrConnect requires us to pass in the JAUS ID for our
	// component as well as the conguration file.  We 
	// get a handle back that's used for subsequent calls.
	// I'm using a subsystem ID (in hex), 
	// and node id 1, component id 1.

	//################################################################
	// CONNECT TO JUNIOR 
	//################################################################
	cout << "\n\n------------------------------------------------";
	cout << "\n\tBEGINNING CONNECTION";
	cout << "\n------------------------------------------------\n\n";
	
	// connect to Junior
	jr_connect_rtn = JrConnect (id, "jr_config.xml", &handle);
	cout << "\nJrConnect Return Value: " << jr_connect_rtn << "\n\n";
	if(jr_connect_rtn == 0)
	{
		cout << "------------------------------------------------\n\n";
		cout << "\n\tSuccessfully Connected To Junior\n";
		cout << "\tHandle: " << handle << "\n";
		cout << "------------------------------------------------\n\n";
	}
	else 
		cout << "\nFailed to connect to Junior.\n\n";

	ros::Duration(1).sleep();

	//################################################################	// SEND QUERY ID
	//################################################################
	// Send Query ID 0x2B00 [as5710, 49]
	bool connection_established = false;
	while( connection_established == false)
	{
		ros::Duration(5).sleep();

		msg_id_init.msg_id 		= 0x2B00;
		msg_id_init.query_type = 2;  // 0 res, 1 sys, 2 subsys, 
		// 3 node, 4 comp, 5 reserved

		if (JrSend(handle, destination, sizeof(msg_id_init), (char*)&msg_id_init) != Ok)
			cout << "\n\t *** Unable to Send Message *** \n\n";
		else 
		{
			cout << "\n *** QUERY IDENTIFICATION sent ***\n";
			connection_established = true;
		}
	}

	//################################################################
	// START INTERACTION
	//################################################################
	// For the rest of the tasks, the COP initatives everything
	while(ros::ok())
	{
		ros::spinOnce();
		//################################################################
		// LOOP AND RECEIVE MESSAGES
		//################################################################
		size = MAX_BUFFER_SIZE;   	// Initialize size

		jr_receive_rtn = JrReceive(handle, &source, &size, buffer);
													//&priority, &flags, &msg_id);
		cout << "\nJrReceive Return Value: " << jr_receive_rtn << "\n\n";
		if (jr_receive_rtn == 0)
		{
			// print info about the message
			cout << "\n\nMSG_ID = " << msg_id;
			cout << "\nReceived " << size << " bytes from " << source << "\n";

			// then do something depending on what the message ID was
			// 
			switch( *((unsigned short*)buffer) )
			{
				//################################################################
				// REPORT LOCAL POSE
				//################################################################
				
				case 0x2403: // msg_id for Query Local Pose, [as6009, 66]
					{
						// send Report Local Pose, 0x4403 [as6009, 72]
						msg_pose.msg_id 	= 0x4403;
						msg_pose.pv 			= 64; 
						msg_pose.yaw 			= scaleToUInt16(get_yaw(), -3.14, 3.14); 

						// Now we send the message to the COP using Junior.  Recall
						// that the COP subsystem id is decimal 90 (0x005A hex)
						if (JrSend(handle, destination, sizeof(msg_pose), (char*)&msg_pose) != Ok)
							cout << "\n\t *** Unable to Send Message *** \n\n";
						else 
							cout << "\n *** REPORT LOCAL POSE sent ***\n";
						break;
					}

					//################################################################
					// REPORT VELOCITY STATE
					//################################################################
				case 0x2404: // msg_id for Query Velocity State
					{
						// Now send Report Velocity State, 0x4404 [as6009, 72]
						msg_vel.msg_id 		= 0x4404;
						msg_vel.pv 				= 1; // 1 for competition 
						// rescale this
						msg_vel.vel_x 		= scaleToUInt32(15, -327.68, 327.67);

						if (JrSend(handle, destination, sizeof(msg_vel), (char*)&msg_vel) != Ok)
							cout << "\n\t *** Unable to Send Message *** \n\n";
						else 
							cout << "\n *** REPORT VELOCITY STATE sent ***\n";
						break;
					}

					//################################################################
					// IF RECEIVE "REPORT IDENTIFICATION" 
					//################################################################
				case 0x4B00: // msg_id for Report ID [as5710, 54] 
					{
						cout << "\n\nReceieved Report Identification\n\n";

						// now I will receive query_id
						bool query_id_received = false;
						while (query_id_received == false)
						{
							size = MAX_BUFFER_SIZE;
							jr_receive_rtn = JrReceive(handle, &source, &size, buffer);
							if ( *((unsigned short*)buffer) == 0x2B00)
								query_id_received = true;
						}

						// now send my own Report Identification back
						// msg_id for Report ID [as5710, 54] 
						msg_report_id.msg_id 					= 0x4B00;
						msg_report_id.query_type			= 2;
						msg_report_id.type						= 10001;
						msg_report_id.byte						= 6;
						string dummy = "SeaBee";

						memcpy(msg_report_id.id, dummy.c_str(), 6); //serialize count before string


						/// needs to be 42 1 1 instead of 42 1 0, 
						// come up in standby instead of ready

						if (JrSend(handle, destination, sizeof(msg_report_id), (char*)&msg_report_id) != Ok)
							cout << "\n\t *** Unable to Send Message *** \n\n";
						else 
							cout << "\n *** REPORT ID MESSAGE sent ***\n";
						break;
					}

					//################################################################
					// SYSTEM MANAGEMENT (next few case statements)
					//################################################################
				case 0x200D: // msg_id for Query Control [as5710, 47] 
					{
						// same byte packing issue
						// send Report Control, 0x400D [as5710, 52]
						msg_ctrl.msg_id				= 0x400D;		 	
						msg_ctrl.ss_id				= 42;
						msg_ctrl.id_stuff			= 0x0101;	
						msg_ctrl.auth_code		= '1'; 

						if (JrSend(handle, destination, sizeof(msg_ctrl), (char*)&msg_ctrl) != Ok)
							cout << "\n\t *** Unable to Send Message *** \n\n";
						else 
							cout << "\n *** REPORT CONTROL MESSAGE sent ***\n";
						break;
					}

				case 0x000D: // msg_id for Request Control [as5710, 41]
					{
						next_byte =  buffer[2];

						cout << "\n\n" << next_byte << "\n\n";

						// send Confirm Control, 0x000F [as5710, 42] 
						msg_cfrm.msg_id					= 0x000F;
						msg_cfrm.response_code	= next_byte; 
						// Response Code:
						// 0 CONTROL ACCEPTED
						// 1 NOT AVAILABLE
						// 2 INSUFFICIENT AUTHORITY

						if (JrSend(handle, destination, sizeof(msg_cfrm), (char*)&msg_cfrm) != Ok)
							cout << "\n\t *** Unable to Send Message *** \n\n";
						else 
							cout << "\n *** CONFIRM CONTROL MESSAGE sent ***\n";
						break;
					}
				case 11008:
					
				case 0x0004:
					{
						state = 0x01;

						msg_status.msg_id		=	0x4002;
						msg_status.status		= state;
						msg_status.reserved = 0;
						break;
					
					}
				case 0x0003:
					{
						state = 0x02;

						msg_status.msg_id		=	0x4002;
						msg_status.status		= state;
						msg_status.reserved = 0;
						break;
					
					}

				case 0x2002: // msg_id for Query Status [as5710, 46]
					{
						// send Report Status, 0x4002 [as5710, 51]

						msg_status.msg_id		=	0x4002;
						msg_status.status		= state;
						msg_status.reserved = 0;
						// Status:
						// 0 INIT
						// 1 READY
						// 2 STANDBY
						// 3 SHUTDOWN
						// 4 FAILURE
						// 5 EMERGENCY
						if (JrSend(handle, destination, sizeof(msg_status), (char*)&msg_status) != Ok)
							cout << "\n\t *** Unable to Send Message *** \n\n";
						else 
							cout << "\n *** REPORT STATUS MESSAGE sent ***\n";
						break;
					}

					//################################################################
					// CAPABILITIES 
					//################################################################
				case 0x2B03: // msg_id for Query Services [as5710, 50]
					{
						// send Report Services, 0x4B03 [as5710, 56]
						// serialize all these inherited messages together
						msg_service.msg_id          = 0x4B03;
						msg_service.node_list				= 1;
						msg_service.node_id					= 1;
						msg_service.component_list	= 1;
						msg_service.component_id		= 1;
						msg_service.instance_id			= 0;
						msg_service.service_list		= 1;
						msg_service.dummy						= 5;

						string dummy_uri = "STUFF";

						memcpy(msg_service.uri, dummy_uri.c_str(), 5); //serialize count before string
						
						msg_service.version_major		= 4;
						msg_service.version_minor		= 2;

						if (JrSend(handle, destination, sizeof(msg_service), (char*)&msg_service) != Ok)
							cout << "\n\t *** Unable to Send Message *** \n\n";
						else 
							cout << "\n *** REPORT SERVICES MESSAGE sent ***\n";
						break;
							
					}
					default: 
					{
						cout << "\n \n \n \n \n" << *((unsigned short*)buffer) << "state: " << state << "\n \n \n \n \n \n" << endl;
					}
			}
		}
		ros::Rate(20).sleep();
		ros::spinOnce();
	}

	//################################################################
	// Clean-up
	//################################################################
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

next_4_bytes =  (buffer[2] << 24);
next_4_bytes |= (buffer[3] << 16);
next_4_bytes |= (buffer[4] << 8);
next_4_bytes |= (buffer[5]);
*/

