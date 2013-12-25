/***************************************************************************
 *  include/uscauv_common/tic_toc.h
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


#ifndef USCAUV_USCAUVCOMMON_TICTOC
#define USCAUV_USCAUVCOMMON_TICTOC

#include <chrono>
#include <string>
#include <ros/ros.h>

namespace uscauv
{
  template <typename __DurationType>
    struct duration_traits;
}

/**
 * Have to do const cast to avoid a compiler warning.
 * This is fine because we only plan on calling with string literals and won't mutate name.
 */
#define USCAUV_DECLARE_DURATION_TRAITS(__DurationType, __DurationName) \
  namespace uscauv { \
  template<> struct duration_traits<__DurationType> { \
    static constexpr char * name = const_cast<char*>(__DurationName); }; }

USCAUV_DECLARE_DURATION_TRAITS( std::chrono::nanoseconds,  "nanoseconds"  );
USCAUV_DECLARE_DURATION_TRAITS( std::chrono::microseconds, "microseconds" );
USCAUV_DECLARE_DURATION_TRAITS( std::chrono::milliseconds, "milliseconds" );
USCAUV_DECLARE_DURATION_TRAITS( std::chrono::seconds,      "seconds"      );
USCAUV_DECLARE_DURATION_TRAITS( std::chrono::minutes,      "minutes"      );
USCAUV_DECLARE_DURATION_TRAITS( std::chrono::hours,        "hours"        );

#define tic								\
  std::chrono::system_clock::time_point __tic_toc_start_time = std::chrono::system_clock::now();

#define toc(__DurationType)						\
  std::chrono::duration_cast<__DurationType>				\
  (std::chrono::system_clock::now() - __tic_toc_start_time).count()  

#define make_toc_stream(__DurationType, __Args)				\
  __Args << " duration: " << toc(__DurationType) << " " << uscauv::duration_traits<__DurationType>::name << "."

#define toc_stream(__DurationType, __Args)				\
  std::cout << make_toc_stream(__DurationType, __Args) << std::endl;

#define toc_info_stream(__DurationType, __Args)		\
  ROS_INFO_STREAM( make_toc_stream(__DurationType, __Args) );

#define toc_debug_stream(__DurationType, __Args)		\
  ROS_DEBUG_STREAM( make_toc_stream(__DurationType, __Args) );

#endif // USCAUV_USCAUVCOMMON_TICTOC
