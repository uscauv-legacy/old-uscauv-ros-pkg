/***************************************************************************
 *  src/xsens_driver.cpp
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

int XsensDriver::connect(uint32_t const & baudrate)
{
  /// Initialize all port info fields to zero (device ID, bus, baud, port name)
  CmtPortInfo target_port = {0, 0, 0, ""};
  XsensResultValue open_result;
  int device_count;
  /// uint32_t
  CmtDeviceId device_id;
  
  /// Set name of the port to scan
  sprintf( target_port.m_portName, "%s", port_.c_str() );

  ROS_INFO("Scanning for connected Xsens devices on port [ %s ] with baud rate [ %s ]...", target_port.m_portName, baud_string_map_[ 0 ].c_str() );
  
  if ( !xsens::cmtScanPort( target_port, 0 ) )
    {
      ROS_ERROR("Failed to find device.");
      return -1;
    }
  
  port_info_ = target_port;

  ROS_INFO("Found device [ %d ] on COM port [ %s ], port number [ %d ] at baud rate [ %s ].", 
	   port_info_.m_deviceId, port_info_.m_portName, port_info_.m_portNr, baud_string_map_[ port_info_.m_baudrate ].c_str() );

  /// If cmtScanPort is unable to connect with the requested baud rate, it will try others and set the m_baudrate field if it finds a match
  if( baudrate != port_info_.m_baudrate )
    ROS_WARN( "Baud rate is different from requested.");

  ROS_INFO("Opening port...");

  /**
   * Open a connection to the device and place it in config mode.
   * Uses default read and write buffer sizes (see cmt3.h)
   */
  open_result = cmt3_.openPort(port_info_.m_portName, port_info_.m_baudrate);

  if ( open_result != XRV_OK )
    {
      ROS_ERROR("Failed to open COM port. [ %s ]", xsensResultText( open_result ) );
      return -1;
    }

  /// If device count isn't set to 1 at this point we have done something horribly wrong.
  device_count = cmt3_.getMtCount();
  /// TODO: Check result value for this function call
  cmt3_.getDeviceId( 1, device_id );
  
  ROS_INFO( "Opened [ %d ] device with ID [ %d ]. The device is now in config mode.", device_count, device_id );

  return 0;
}


int XsensDriver::settingsFromDevice()
{
  for (_LambdaArray::const_iterator getter_it = cmt_getter_functions_.begin();
       getter_it != cmt_getter_functions_.end(); ++getter_it)
    {
      (*getter_it)();
    }

  // ROS_INFO("Got magnetic declination [ %f ]", cmt_info_.magnetic_declination_);

  return 0;
}
