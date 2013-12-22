/***************************************************************************
 *  include/uscauv_common/param_writer.h
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


#ifndef USCAUV_USCAUVCOMMON_PARAMWRITER
#define USCAUV_USCAUVCOMMON_PARAMWRITER

// ROS
#include <ros/ros.h>
#include <uscauv_common/macros.h>

namespace uscauv
{

  namespace param
  {
    
    /** 
     * Save a parameter on the parameter server to file. Does this as a hack by calling rosparam 
     * 
     * @param nh Nodehandle whose namespace we will use to search for the parameter
     * @param param parameter to save
     * @param file File to write to
     * 
     * @return False if successful, true otherwise
     */
    bool save( ros::NodeHandle const & nh, std::string const & param, std::string const & file );

    /** 
     * Save a parameter on the parameter server to file. Does this as a hack by calling rosparam 
     * 
     * @param nh Nodehandle whose namespace we will use to search for the parameter
     * @param param parameter to save
     * @param package ROS package to save the file relative to
     * @param file File to write to
     * 
     * @return False if successful, true otherwise
     */
    bool save( ros::NodeHandle const & nh, std::string const & param, 
	       std::string const & package, std::string const & file );

  }
    
} // uscauv

#endif // USCAUV_USCAUVCOMMON_PARAMWRITER
