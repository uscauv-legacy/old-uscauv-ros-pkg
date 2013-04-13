/***************************************************************************
 *  xsens_driver.h
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

#ifndef USCAUV_XSENSDRIVER_XSENSDRIVER_H
#define USCAUV_XSENSDRIVER_XSENSDRIVER_H

/// ROS
#include <ros/ros.h>

#include "cmtdef.h"
#include "xsens_time.h"
#include "xsens_list.h"
#include "cmtscan.h"
#include "cmt3.h"


typedef std::map<int, std::string> _MacroStringMap;

static _MacroStringMap const BAUD_NAMES_
{
  {CMT_BAUD_RATE_4800,  "4800"},
  {CMT_BAUD_RATE_9600,  "9600"},
  {CMT_BAUD_RATE_19K2,  "19k2"},
  {CMT_BAUD_RATE_38K4,  "38k4"},
  {CMT_BAUD_RATE_57K6,  "57k6"},
  {CMT_BAUD_RATE_115K2, "115k2"},
  {CMT_BAUD_RATE_230K4, "230k4"},
  {CMT_BAUD_RATE_460K8, "460k8"},
  {CMT_BAUD_RATE_921K6, "921k6"},
};


class XSensDriver
{
 private:
  /* Put the IMU data that will be pulled from the device every time update() is called here */

 private:
  /* IMU Config parameters */
  /* Should these be in a struct? */

 private:
  /* Misc parameters */
  bool connected_;
  
  std::string port_;
  
 private:
  /* maps */
  _MacroStringMap baud_string_map_;
  
 private:
  /* Xsens Parameters */

  /// This class encapsulates the xsens communications functionality
  xsens::Cmt3 cmt3_;

  /// If connect() is successful, contains baud rate, COM port, etc.
  xsens::CmtPortInfo port_info_;

 public:

 XsensDriver(std::string const & port):
  port_(port),
  baud_string_map_( BAUD_NAMES_ )
  {
    
  }

  /** 
   * Connect to an Xsens IMU on the port that this class was constructed with.
   * 
   * @return Zero on success, non-zero otherwise
   */
  /// TODO: Find a better way of ensuring baud rate safety (write to file?)
  int connect(uint32_t const & baudrate = CMT_BAUD_RATE_921K6);

  int settingsFromDevice();

  /// Just calls connect -> settingsFromDevice
  int init();

};

#endif // USCAUV_XSENSDRIVER_XSENSDRIVER_H
