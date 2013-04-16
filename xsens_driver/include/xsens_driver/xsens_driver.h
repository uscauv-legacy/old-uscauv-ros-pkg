/***************************************************************************
 *  /include/xsens_driver/xsens_driver.h
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

#include <uscauv_utilities/stream.h>

typedef std::map<int, std::string> _MacroStringMap;
typedef std::vector<std::function<void()> > _LambdaArray;

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
  /// In general, providing 0 as an argument to the cmt libs will make them check all baud rates
  {0, "All"}
};

/// TODO: Figure out how to make printing work when there is more than one
/// variadic arg
/// Or when the return value is not an argument to the function call
/// probably need to switch to templates to do this
/// Is it possible to call a function on __Args that will convert Arg1, Arg2, ... to Arg1 << ", " << Arg2 << ", " << ...


#define GET_CMT_GETTER_LAMBDA(__FunctionCall, __CMTInstance, __DataString, __Args...) \
  [&](){XsensResultValue res =						\
      __CMTInstance.__FunctionCall(__Args);				\
    if (res != XRV_OK)							\
      ROS_ERROR("Unable to fetch device parameter [ %s ]. Error: [ %s ]", __DataString, xsensResultText( res )); \
    else ROS_INFO_STREAM("Retrieved parameter [ " << __DataString	\
			 << " ] with value [ " << make_stream(std::string(", "), __Args) << " ]");}

struct CmtInfo
{
  double heading_offset_;
  double magnetic_declination_;
  double sample_frequency_;
  double gravity_magnitude_;

  uint8_t current_scenario_type_;
  uint8_t current_scenario_version_;
  CmtScenario available_scenarios_[16];

  CmtVector lat_lon_alt_;
  CmtMatrix object_alignment_matrix_;
  
  uint16_t processing_flags_;
  uint16_t transmission_delay_;

  char product_code_[128];
  CmtVersion firmware_revision_;

};

class XsensDriver
{
 private:
  /* Put the IMU data that will be pulled from the device every time update() is called here */

 private:
  /* IMU Config parameters */
  CmtInfo cmt_info_;
  _LambdaArray cmt_getter_functions_;

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
  CmtPortInfo port_info_;

 public:

 XsensDriver(std::string const & port):
  port_(port),
  baud_string_map_( BAUD_NAMES_ )
  {
    /* std::function<void()> getmagdec_f = GET_CMT_GETTER_LAMBDA( getMagneticDeclination, cmt3_, "Magnetic Declination", cmt_info_.magnetic_declination_); */
    /* cmt_getter_functions_.push_back( getmagdec_f ); */

    /// TODO: Sample frequency (returns value ), current scenarios (print will fail due to two args), available scenarios (needs print overloaded)
    /// Lat lon alt (need to overload CmtVector), alignment (need to overload CmtMatrix), 
    /// Firmware revision (overload)
    _LambdaArray getters
    {
      GET_CMT_GETTER_LAMBDA( getHeading, cmt3_, "Heading Offset", cmt_info_.heading_offset_),
      GET_CMT_GETTER_LAMBDA( getMagneticDeclination, cmt3_, "Magnetic Declination", cmt_info_.magnetic_declination_),
      GET_CMT_GETTER_LAMBDA( getGravityMagnitude, cmt3_, "Gravity Magnitude", cmt_info_.gravity_magnitude_),
      GET_CMT_GETTER_LAMBDA( getProcessingFlags, cmt3_, "Processing Flags", cmt_info_.processing_flags_),
      GET_CMT_GETTER_LAMBDA( getTransmissionDelay, cmt3_, "Transmission Delay", cmt_info_.transmission_delay_),
      GET_CMT_GETTER_LAMBDA( getProductCode, cmt3_, "Product Code", cmt_info_.product_code_),
      GET_CMT_GETTER_LAMBDA( getScenario, cmt3_, "Current Scenario", cmt_info_.current_scenario_type_, cmt_info_.current_scenario_version_)
   };

    cmt_getter_functions_ = getters;

  }

  /** 
   * Connect to an Xsens IMU on the port that this class was constructed with.
   * 
   * @return Zero on success, non-zero otherwise
   */
  /// TODO: Find a better way of ensuring baud rate safety (write to file?)
  int connect(uint32_t const & baudrate = CMT_BAUD_RATE_921K6);

  int settingsFromDevice();

  int settingsToDevice();

  /// Just calls connect -> settingsFromDevice -> gotoMeasurement
  /// TODO: Check that avaiable output modes match our desired mode
  int init();

};

#endif // USCAUV_XSENSDRIVER_XSENSDRIVER_H
