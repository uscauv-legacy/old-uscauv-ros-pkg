/***************************************************************************
 *  include/base_libs/runable_policy.h
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

#ifndef BASE_LIBS_BASE_LIBS_RUNABLE_POLICY_H_
#define BASE_LIBS_BASE_LIBS_RUNABLE_POLICY_H_

#include <base_libs/node_handle_policy.h>
#include <ros/rate.h>

namespace base_libs
{


BASE_LIBS_DECLARE_POLICY( Runable, NodeHandlePolicy )

BASE_LIBS_DECLARE_POLICY_CLASS( Runable )
{
	BASE_LIBS_MAKE_POLICY_NAME( Runable )
	
protected:
	ros::Rate * loop_rate_;
	bool run_;
	
	BASE_LIBS_DECLARE_POLICY_CONSTRUCTOR( Runable ),
		run_( false )
	{
		printPolicyActionStart( "create", this );
		
		preInit();
		
		printPolicyActionDone( "create", this );
	}
	
	void preInit()
	{
		auto & nh_rel = NodeHandlePolicy::getNodeHandle();
		
		loop_rate_ = new ros::Rate( ros::ParamReader<double, 1>::readParam( nh_rel, "loop_rate", 10 ) );
	}
	
	~RunablePolicy()
	{
		if( loop_rate_ ) delete loop_rate_;
	}
	
	virtual void spinFirst(){}
	
	virtual void spinOnce(){}
	
	virtual void spin()
	{
		run_ = true;
		
		PRINT_INFO( "--------------------" );
		PRINT_INFO( ">>>>> Starting pre-spin..." );
		
		spinFirst();
		
		PRINT_INFO( "--------------------" );
		PRINT_INFO( "<<<<< Done pre-spin" );
		
		PRINT_INFO( "--------------------" );
		PRINT_INFO( ">>>>> Spinning at %f Hz...", 1.0 / loop_rate_->expectedCycleTime().toSec() );
		
		while( run_ && ros::ok() )
		{
			spinOnce();
			ros::spinOnce();
			if( loop_rate_ ) loop_rate_->sleep();
		}
		
		PRINT_INFO( "<<<<< Main loop finished" );
	}
	
	virtual void interrupt()
	{
		PRINT_INFO( ">>>>> Interrupting main loop..." );
		run_ = false;
	}
};

}

#endif // BASE_LIBS_BASE_LIBS_RUNABLE_POLICY_H_
