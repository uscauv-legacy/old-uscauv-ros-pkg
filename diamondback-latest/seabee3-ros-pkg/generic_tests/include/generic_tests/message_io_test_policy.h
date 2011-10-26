/***************************************************************************
 *  include/generic_tests/message_io_test_policy.h
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

#ifndef GENERIC_TESTS_GENERIC_TESTS_MESSAGE_IO_TEST_POLICY_H_
#define GENERIC_TESTS_GENERIC_TESTS_MESSAGE_IO_TEST_POLICY_H_

#include <base_libs/node_handle_policy.h>
#include <base_libs/multi_subscriber.h>
#include <base_libs/multi_publisher.h>
#include <base_libs/macros.h>
#include <generic_tests/test.h>

namespace base_libs
{

BASE_LIBS_DECLARE_POLICY( MessageIOTest, NodeHandlePolicy )

template<class __InputMessage, class __OutputMessage>
BASE_LIBS_DECLARE_POLICY_CLASS( MessageIOTest )
{
	BASE_LIBS_MAKE_POLICY_NAME( MessageIOTest )

public:
	typedef MessageIOTestPolicy<__InputMessage, __OutputMessage> _MessageIOTestPolicy;

private:
	ros::MultiSubscriber<> multi_sub_;
	ros::MultiPublisher<> multi_pub_;
	
	BASE_LIBS_DECLARE_POLICY_CONSTRUCTOR( MessageIOTest )
	{
		//
	}
	
	// input<__InputMessage> --> [node to test] --> output<__OutputMessage>
	BASE_LIBS_ENABLE_INIT
	{
		auto & nh_rel = NodeHandlePolicy::getNodeHandle();
		
		if( getMetaParamDef<bool>( "enable_publisher_param", false, args... ) )
		{		
			const std::string input_topic_name_param = getMetaParamDef<std::string>( "input_topic_name_param", "input_topic_name", args... );
			const std::string input_topic_name = ros::ParamReader<std::string, 1>::readParam( nh_rel, input_topic_name_param, "input" );
			multi_pub_.addPublishers<__InputMessage>( nh_rel, { input_topic_name } );
		}
		
		const std::string output_topic_name_param = getMetaParamDef<std::string>( "output_topic_name_param", "input_topic_name", args... );
		const std::string output_topic_name = ros::ParamReader<std::string, 1>::readParam( nh_rel, output_topic_name_param, "output" );
		multi_sub_.addSubscriber( nh_rel, output_topic_name, &_MessageIOTestPolicy::testOutputCB_0, this );
	}
	
	BASE_LIBS_DECLARE_MESSAGE_CALLBACK( testOutputCB_0, typename __OutputMessage )
	{
		testOutputCB( msg );
	}
	
	virtual BASE_LIBS_DECLARE_MESSAGE_CALLBACK( testOutputCB, typename __OutputMessage ){}
};

}

#endif // GENERIC_TESTS_GENERIC_TESTS_MESSAGE_IO_TEST_POLICY_H_
