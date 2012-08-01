/***************************************************************************
 *  include/generic_tests/message_io_test.h
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

#ifndef GENERIC_TESTS_GENERIC_TESTS_MESSAGE_IO_TEST_H_
#define GENERIC_TESTS_GENERIC_TESTS_MESSAGE_IO_TEST_H_

#include <quickdev/node.h>
#include <generic_tests/message_io_test_policy.h>

template<class __InputMessage, class __OutputMessage>
class MessageIOTestNode : public quickdev::Node< quickdev::MessageIOTestPolicy<__InputMessage, __OutputMessage> >
{
public:
	typedef MessageIOTestNode<__InputMessage, __OutputMessage> _MessageIOTestNode;
	typedef quickdev::MessageIOTestPolicy<__InputMessage, __OutputMessage> _MessageIOTestPolicy;
	typedef quickdev::Node<_MessageIOTestPolicy> _Node;

public:
	template<class... __Args>
	MessageIOTestNode( __Args&&... args )
	:
		_Node( args... )
	{
		//
	}

	// called by message_io_test_policy::testOutputCB_0
	QUICKDEV_DECLARE_MESSAGE_CALLBACK( testOutputCB, typename __OutputMessage )
	{
		PRINT_INFO( "Got message" );

		// compare outputs / log results here
	}
};

#endif // GENERIC_TESTS_GENERIC_TESTS_MESSAGE_IO_TEST_H_
