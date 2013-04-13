/***************************************************************************
 *  ../../src/xsens_driver.cpp
 *  --------------------
 *
 *  Copyright (c) 2013, Dylan Foster
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are
 *  met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following disclaimer
 *    in the documentation and/or other materials provided with the
 *    distribution.
 *  * Neither the name of USC AUV nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 **************************************************************************/

#include <xsens_driver/xsens_driver.h>

int XsensDriver::connect(uint32_t baudrate)
{
  xsens::CmtPortInfo target_port;
  xsens::XsensResultValue open_result;
  int device_count;
  int device_id;
  
  ROS_INFO("Scanning for connected Xsens devices on port [ %s ] with baud rate [ %d ]...", port_.c_str(), baudrate);
  
  if ( !xsens::cmtScanPort( target_port, baudrate ) )
    {
      ROS_ERROR("Failed to find device.");
      return -1;
    }
  
  port_info_ = target_port;

  ROS_INFO("Found device [ %d ] on COM port [ %s ], port number [ %d ] at baud rate [ %s ].", 
	   port_info_.m_deviceId, port_info_.m_portName, port_info_.m_portNr, baud_string_map_[ port_info.m_baudrate ].c_str() );

  /// If cmtScanPort is unable to connect with the requested baud rate, it will try others and set the m_baudrate field if it finds a match
  if( baudrate != port_info_.m_baudrate )
    ROS_WARN( "Baud rate is different from requested.");

  ROS_INFO("Opening port...");

  /**
   * Open a connection to the device and place it in config mode.
   * Uses default read and write buffer sizes (see cmt3.h)
   */
  open_result = cmt3_.openPort(port_info_.m_portName, port_info_.m_baudrate);

  if ( open_result != xsens::XRV_OK )
    {
      ROS_ERROR("Failed to open COM port.");
      return -1;
    }

  /// If device count isn't set to 1 at this point we have done something horribly wrong.
  device_count = cmt3_.getMtCount();
  device_id    = cmt3_.getDeviceId();
  
  ROS_INFO( "Opened [ %d ] device with ID [ %d ]. The device is now in config mode.", device_count, device_id );

  return 0;
}


int XsensDriver::settingsFromDevice()
{
  
};
