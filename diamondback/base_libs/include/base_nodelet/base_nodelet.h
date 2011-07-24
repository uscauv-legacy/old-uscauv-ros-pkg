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
#include <functional>

template<class __DataType>
class BaseNodelet: public nodelet::Nodelet
{
public:
	typedef __DataType _DataType;
	typedef BaseNodelet<__DataType> _BaseNodelet;
	typedef std::function<void()> InterruptCallback;

protected:
	ros::NodeHandle nh_;
	ros::NodeHandle nh_local_;
	InterruptCallback interrupt_callback_;
	__DataType * data_;
	std::thread * m_thread_;

	volatile bool running_;

public:
	BaseNodelet() :
	nodelet::Nodelet(), data_( NULL ), m_thread_( NULL ), running_( false )
	{
		ROS_INFO( "Setting up base_nodelet..." );
		ROS_INFO( "Done setting up base_nodelet" );
	}

	virtual ~BaseNodelet()
	{
		ROS_INFO( "Shutting down base_nodelet..." );
		if ( running_ && m_thread_ )
		{
			ROS_INFO( "Interrupting blocking processes..." );
			if( interrupt_callback_ )
			{
				interrupt_callback_();
				ROS_INFO( "Done interrupting blocking processes" );
			}
			else ROS_INFO( "No interrupt callback registered" );

			ROS_INFO( "Stopping thread..." );
			if( m_thread_ ) m_thread_->join();
			ROS_INFO( "Done stopping thread" );

			ROS_INFO( "Deleting data..." );
			if( data_ ) delete data_;
			ROS_INFO( "Done deleting data" );
		}
		if( m_thread_ ) delete m_thread_;
		ROS_INFO( "Done shutting down base_nodelet" );
	}

	template<class __CallerType>
	void registerInterrupt( void(__CallerType::*callback)(), __CallerType* caller )
	{
		interrupt_callback_ = std::bind( callback, caller );
	}

	void onInit()
	{
		ROS_INFO( "Initializing..." );

		ROS_INFO( "Getting nodehandles..." );
		nh_ = getNodeHandle();
		nh_local_ = getPrivateNodeHandle();
		ROS_INFO( "Done getting nodehandles" );

		ROS_INFO( "Constructing data..." );
		constructData();
		ROS_INFO( "Done constructing data" );


		ROS_INFO( "Starting thread..." );
		running_ = true;
		m_thread_ = new std::thread( &BaseNodelet::spin,
		                             this );
		ROS_INFO( "Done starting thread" );

		ROS_INFO( "Done initializing" );
	}

	virtual void constructData() = 0;

	virtual void spin() = 0;
};

#endif /* BASE_NODELET_H_ */
