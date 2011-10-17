/***************************************************************************
 *  include/base_libs/timed_policy.h
 *  --------------------
 * 
 *  Copyright (c) 2011, Edward T. Kaszubski ( ekaszubski@gmail.com )
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
 *  * Neither the name of seabee3-ros-pkg nor the names of its
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

#ifndef BASE_LIBS_BASE_LIBS_TIMED_POLICY_H_
#define BASE_LIBS_BASE_LIBS_TIMED_POLICY_H_

#include <base_libs/policy.h>
#include <ros/time.h>

namespace base_libs
{

BASE_LIBS_DECLARE_POLICY( Timed, Policy )

BASE_LIBS_DECLARE_POLICY_CLASS( Timed )
{
	BASE_LIBS_MAKE_POLICY_NAME( Timed )

protected:
	ros::Time last_time_;
	ros::Time now_;
	double dt_;
	
	BASE_LIBS_DECLARE_POLICY_CONSTRUCTOR( Timed ),
		dt_( 0 ),
		now_( 0 ),
		last_time_( 0 )
	{
		printPolicyActionStart( "create", this );
		printPolicyActionDone( "create", this );
	}
	
	BASE_LIBS_ENABLE_UPDATE
	{
		if( now_.toSec() == 0 )
		{
			now_ = ros::Time::now();
			last_time_ = now_;
			return;
		}
		
		last_time_ = now_;
		now_ = ros::Time::now();
		dt_ = ( now_ - last_time_ ).toSec();
	}
};

}

#endif // BASE_LIBS_BASE_LIBS_TIMED_POLICY_H_
