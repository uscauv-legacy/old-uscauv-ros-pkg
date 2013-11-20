/*******************************************************************************
  *
  *      xsens_driver
  *
  *      Copyright (c) 2010, Edward T. Kaszubski (ekaszubski@gmail.com)
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
  *      * Neither the name of the USC Underwater Robotics Team nor the names of its
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

#ifndef XSensDriver_C
#define XSensDriver_C

#include <xsens_driver/xsens_driver.h>

#include <ros/ros.h>

#include <iostream>
#include <sstream>

using namespace xsens;

XSensDriver::XSensDriver( std::string const & port )
    :
  port_( port ),
  res( XRV_OK ),
  inited( false )
{
  //
}

XSensDriver::~XSensDriver()
{
  delete packet;
}

bool XSensDriver::initMe()
{
  unsigned long mtCount = doHardwareScan( cmt3, deviceIds );

  if ( mtCount == 0 )
    {
      ROS_FATAL("No motion trackers found, quitting.");
      cmt3.closePort();
      inited = true;
      return false;
    }

  // Get the available scenarios for this IMU and print them out
  CmtScenario scenarios[CMT_MAX_SCENARIOS_IN_MT + 1];
  if ( cmt3.getAvailableScenarios( scenarios ) != XRV_OK )
    {
      ROS_ERROR("Error getting IMU scenarios! Please check the IMU!");
    }
  else
    {
      for ( int scenIdx = 0; scenIdx < CMT_MAX_SCENARIOS_IN_MT + 1; scenIdx++ )
        {
	  CmtScenario scenario = scenarios[scenIdx];
	  if ( scenario.m_type == 0 ) break;
	  ROS_INFO_STREAM( "Scenario #" << scenIdx << " m_type=[" << (int) scenario.m_type << "] : " << scenario.m_label );
        }
    }

  // Set the current filtering scenario
  // Modify this scenarioType variable to set the IMU's filtering scenario.
  uint8_t scenarioType = 6;
  if ( cmt3.setScenario( scenarioType ) == XRV_OK ) ROS_INFO_STREAM("Sucessfully set scenario type to [" << (int) scenarioType << "]" );
  else ROS_ERROR_STREAM( "Failed to set scenario type! Please check the IMU!");

  // Set the output mode
  mode = CMT_OUTPUTMODE_CALIB | CMT_OUTPUTMODE_ORIENT;
  settings = CMT_OUTPUTSETTINGS_ORIENTMODE_EULER;
  doMtSettings( cmt3, mode, settings, deviceIds );

  // Print out the current device configuration to make sure everything's ok
  CmtDeviceConfiguration configuration;
  if ( cmt3.getConfiguration( configuration ) != XRV_OK )
    {
      ROS_ERROR_STREAM( "Configuration query failed! Check IMU!");
    }
  else
    {
      ROS_INFO("Got Configuration:" );
      for ( uint16_t devIdx = 0; devIdx < configuration.m_numberOfDevices; devIdx++ )
        {
	  ROS_INFO_STREAM( " - Device[" << devIdx << "]");
	  ROS_INFO_STREAM( " |- m_currentScenario:  " << configuration.m_deviceInfo[0].m_currentScenario);
	  ROS_INFO_STREAM( " |- m_filterType:       " << (int) configuration.m_deviceInfo[0].m_filterType);
	  ROS_INFO_STREAM( " |- m_filterMajor:      " << (int) configuration.m_deviceInfo[0].m_filterMajor);
	  ROS_INFO_STREAM( " |- m_filterMinor:      " << (int) configuration.m_deviceInfo[0].m_filterMinor);
        }
      ROS_INFO( "Everything looks good, we're ready to roll" );
    }

  // Initialize packet for data
  packet = new Packet( (unsigned short) mtCount, cmt3.isXm() );

  inited = true;
  return true;
}

bool XSensDriver::updateData()
{
  if ( ( !inited && !initMe() ) || res != XRV_OK ) return false;

  cmt3.waitForDataMessage( packet );

  if ( ( mode & CMT_OUTPUTMODE_CALIB ) != 0 )
    {
      caldata = packet->getCalData( 0 );

      accel_.x = caldata.m_acc.m_data[0];
      accel_.y = caldata.m_acc.m_data[1];
      accel_.z = caldata.m_acc.m_data[2];

      gyro_.x = caldata.m_gyr.m_data[0];
      gyro_.y = caldata.m_gyr.m_data[1];
      gyro_.z = caldata.m_gyr.m_data[2];

      mag_.x = caldata.m_mag.m_data[0];
      mag_.y = caldata.m_mag.m_data[1];
      mag_.z = caldata.m_mag.m_data[2];

    }

  if ( ( mode & CMT_OUTPUTMODE_ORIENT ) != 0 )
    {

      switch ( settings & CMT_OUTPUTSETTINGS_ORIENTMODE_MASK )
        {

        case CMT_OUTPUTSETTINGS_ORIENTMODE_EULER:
	  // Output: Euler
	  euler_data = packet->getOriEuler( 0 );

	  ori_.x = euler_data.m_roll;
	  ori_.y = euler_data.m_pitch;
	  ori_.z = euler_data.m_yaw;

	  break;
        }

      if ( ( mode & CMT_OUTPUTMODE_POSITION ) != 0 )
        {
	  if ( packet->containsPositionLLA() )
            {
	      /* output position */
	      /*              CmtVector positionLLA = packet->getPositionLLA();
			      if (res != XRV_OK)
			      {

			      }

			      for (int i = 0; i < 2; i++)
			      {
			      double deg = positionLLA.m_data[0];
			      double min = (deg - (int) deg) * 60;
			      double sec = (min - (int) min) * 60;

			      }*/

            }
        }
    }

  return true;
}

//////////////////////////////////////////////////////////////////////////
// doHardwareScan
//
// Checks available COM ports and scans for MotionTrackers
int XSensDriver::doHardwareScan( xsens::Cmt3 &cmt3, CmtDeviceId deviceIds[] )
{
  XsensResultValue res;
  List<CmtPortInfo> portInfo;
  unsigned long portCount = 0;
  int mtCount;

  ROS_INFO( "Scanning for connected Xsens devices..." );

  //    xsens::cmtScanPorts( portInfo );
  CmtPortInfo target_port = { 0, 0, 0, "" };
  sprintf( target_port.m_portName, "%s", port_.c_str() );

  if( cmtScanPort( target_port, target_port.m_baudrate ) ) portInfo.append( target_port );

  portCount = portInfo.length();
  ROS_INFO("Done");

  if ( portCount == 0 )
    {
      ROS_FATAL( "No MotionTrackers found" );
      return 0;
    }

  for ( int i = 0; i < (int) portCount; i++ )
    {
      ROS_INFO_STREAM("Using COM port at [ " << portInfo[i].m_portName << " ] ");
      std::string baud;
      
      switch ( portInfo[i].m_baudrate )
        {
        case B9600:
	  baud =  "9k6";
	  break;
        case B19200:
	  baud =  "19k2";
	  break;
        case B38400:
	  baud =  "38k4";
	  break;
        case B57600:
	  baud =  "57k6";
	  break;
        case B115200:
	  baud =  "115k2";
	  break;
        case B230400:
	  baud =  "230k4";
	  break;
        case B460800:
	  baud =  "460k8";
	  break;
        case B921600:
	  baud =  "921k6";
	  break;
        default:
	  baud =  portInfo[i].m_baudrate;
        }
      ROS_INFO_STREAM("Baud rate [ " << baud << " ]");
    }

  ROS_INFO("Opening ports...");
  //open the port which the device is connected to and connect at the device's baudrate.
  for ( int p = 0; p < (int) portCount; p++ )
    {
      res = cmt3.openPort( portInfo[p].m_portName, portInfo[p].m_baudrate );
      //      EXIT_ON_ERROR(res,"cmtOpenPort");
    }
  ROS_INFO("Done.");

  //get the Mt sensor count.
  ROS_INFO_STREAM("Retrieving MotionTracker count (excluding attached Xbus Master(s))" );
  mtCount = cmt3.getMtCount();
  ROS_INFO_STREAM("MotionTracker count: " << mtCount );

  // retrieve the device IDs
  ROS_INFO_STREAM("Retrieving MotionTrackers device ID(s)" );
  for ( int j = 0; j < mtCount; j++ )
    {
      res = cmt3.getDeviceId( (unsigned char) ( j + 1 ), deviceIds[j] );
      //      EXIT_ON_ERROR(res,"getDeviceId");
      ROS_INFO_STREAM("Device ID at busId " << j + 1 << "," << (long) deviceIds[j] );
    }

  return mtCount;
}

//////////////////////////////////////////////////////////////////////////
// doMTSettings
//
// Set user settings in MTi/MTx
// Assumes initialized global MTComm class
void XSensDriver::doMtSettings( xsens::Cmt3 &cmt3, CmtOutputMode &mode, CmtOutputSettings &settings, CmtDeviceId deviceIds[] )
{
  XsensResultValue res;
  unsigned long mtCount = cmt3.getMtCount();
  
  // set sensor to config sate
  res = cmt3.gotoConfig();

  unsigned short sampleFreq;
  sampleFreq = 100;//cmt3.getSampleFrequency();

  ROS_INFO_STREAM("sampling at " << sampleFreq );

  // set the device output mode for the device(s)
  ROS_INFO_STREAM("Configuring your mode selection" );

  for ( unsigned int i = 0; i < mtCount; i++ )
    {
      CmtDeviceMode deviceMode( mode, settings, sampleFreq );
      if ( ( deviceIds[i] & 0xFFF00000 ) != 0x00500000 )
        {
	  // not an MTi-G, remove all GPS related stuff
	  deviceMode.m_outputMode &= 0xFF0F;
        }
      res = cmt3.setDeviceMode( deviceMode, true, deviceIds[i] );
    }

  // start receiving data
  res = cmt3.gotoMeasurement();
}
#endif
