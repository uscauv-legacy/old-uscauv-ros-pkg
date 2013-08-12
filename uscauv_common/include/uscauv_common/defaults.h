/***************************************************************************
 *  include/uscauv_common/defaults.h
 *  --------------------
 *
 *  Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Dylan Foster (turtlecannon@gmail.com)
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


#ifndef USCAUV_USCAUVCOMMON_DEFAULTS
#define USCAUV_USCAUVCOMMON_DEFAULTS

// ROS
#include <ros/ros.h>

namespace uscauv
{
  
  namespace defaults
  {
    
    static char const * const WORLD_LINK = "/world";
    static char const * const CM_LINK = "robot/structures/cm";
    static char const * const CV_LINK = "robot/structures/cv";
    static char const * const MEASUREMENT_LINK = "robot/controls/measurement";
    static char const * const DESIRED_LINK = "robot/controls/desired";
    static char const * const IMU_LINK = "robot/sensors/imu";
    static char const * const DEPTH_LINK = "robot/sensors/depth";

    static char const * const STRUCTURE_PREFIX = "robot/structures";
    static char const * const THRUSTER_PREFIX = "robot/thrusters";
    static char const * const CAMERA_PREFIX = "robot/cameras";
    static char const * const SENSOR_PREFIX = "robot/sensors";

    static char const * const DEPTH_TOPIC = "robot/sensors/depth";
    static char const * const KILLSWITCH_TOPIC = "robot/sensors/kill_switch";

  } // defaults
    
} // uscauv

#endif // USCAUV_USCAUVCOMMON_DEFAULTS
