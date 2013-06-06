/***************************************************************************
 *  include/uscauv_common/timing.h
 *  --------------------
 *
 *  Software License Agreement (BSD License)
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


#ifndef USCAUV_USCAUVCOMMON_TIMING
#define USCAUV_USCAUVCOMMON_TIMING

#include <ros/ros.h>

#include <thread>
#include <functional>
#include <chrono>
#include <mutex>

namespace uscauv
{

  /// TODO: Add restart function
template<class __DurationType>
class AsynchronousTimer
{
private:
  __DurationType countdown_;
  std::function<void(void)> callback_;
  bool running_;
  std::mutex timer_state_mutex_;
  std::thread countdown_thread_;

public:
  template< class... __BindArgs>
  AsynchronousTimer( __BindArgs&&... bind_args ): running_(false)
  {
    callback_ = std::bind( std::forward<__BindArgs>( bind_args )... );
  }

  void start(__DurationType countdown_length)
  {
    countdown_ = countdown_length;
    
    timer_state_mutex_.lock();
    running_ = true; 
    timer_state_mutex_.unlock();
    countdown_thread_ = std::thread( &AsynchronousTimer::countdownThread, this );
  }

  void stop()
  {
    timer_state_mutex_.lock();
    running_ = false;
    timer_state_mutex_.unlock();
    countdown_thread_.join();
  }
  
  bool running()
  {
    std::lock_guard<std::mutex> lock(timer_state_mutex_);
    return running_;
  }
  
  void countdownThread()
  {
    /* std::cout << "started thread" << std::endl; */
    
    while( countdown_ != __DurationType::zero() )
      {
	/* ROS_INFO_STREAM("thread is counting down " << countdown_.count() ); */

	if( timer_state_mutex_.try_lock() )
	  {
	    if ( !running_ )
	      {
		timer_state_mutex_.unlock();
		return;
	      }
	    timer_state_mutex_.unlock();
	  }

	std::this_thread::sleep_for( __DurationType(1) );
	--countdown_;

      }

    /// call the external callback
    callback_();

    timer_state_mutex_.lock();
    running_ = false;
    /* std::cout << "thread finished" << std::endl; */
    timer_state_mutex_.unlock();
    
  }

};

}

#endif // USCAUV_USCAUVCOMMON_TIMING
