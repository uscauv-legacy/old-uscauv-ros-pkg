/*******************************************************************************
 *
 *      base_nodelet
 * 
 *      Copyright (c) 2011, Edward T. Kaszubski ( ekaszubski@gmail.com )
 *      All rights reserved.
 *
 *      Redistribution and use in source and binary forms, with or without
 *      modification, are permitted provided that the following conditions are
 *      met:
 *      
 *      * Redistributions of source code must retain the above copyright
 *        notice, this list of conditions and the following disclaimer.
 *      * Redistributions in binary form must reproduce the above
 *        copyright notice, this list of conditions and the following disclaimer
 *        in the documentation and/or other materials provided with the
 *        distribution.
 *      * Neither the name of "base_libs-RelWithDebInfo@base_libs" nor the names of its
 *        contributors may be used to endorse or promote products derived from
 *        this software without specific prior written permission.
 *      
 *      THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *      "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *      LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *      A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *      OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *      SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *      LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *      DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *      THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *      (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *      OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *******************************************************************************/

#ifndef BASE_NODELET_H_
#define BASE_NODELET_H_

#include <nodelet/nodelet.h>
#include <thread>

template<class _DataType>
class BaseNodelet: public nodelet::Nodelet
{
protected:
	ros::NodeHandle nh_;
	ros::NodeHandle nh_local_;
	_DataType * data_;
	std::thread * m_thread_;

	volatile bool running_;

public:
	BaseNodelet() :
	nodelet::Nodelet(), running_( false )
	{

	}

	virtual ~BaseNodelet()
	{
		delete data_;
		if ( running_ && m_thread_ )
		{
			m_thread_->detach();
		}
		delete m_thread_;
	}

	void onInit()
	{
		nh_ = getNodeHandle();
		nh_local_ = getPrivateNodeHandle();

		constructData();

		running_ = true;
		m_thread_ = new std::thread( &BaseNodelet::spin,
		                             this );
	}

	virtual void constructData() = 0;

	virtual void spin() = 0;
};

#endif /* BASE_NODELET_H_ */
