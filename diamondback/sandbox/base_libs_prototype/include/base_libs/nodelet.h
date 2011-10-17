/***************************************************************************
 *  include/base_libs/nodelet.h
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

#ifndef BASE_LIBS_BASE_LIBS_NODELET_H_
#define BASE_LIBS_BASE_LIBS_NODELET_H_

#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <thread>
#include <functional>
#include <base_libs/console.h>
#include <base_libs/auto_bind.h>

namespace base_libs
{

template<class __DataType>
class Nodelet: public nodelet::Nodelet
{
public:
	typedef __DataType _DataType;
	typedef Nodelet<__DataType> _Nodelet;
	typedef std::function<void()> _InterruptCallback;

protected:
	ros::NodeHandle nh_;
	ros::NodeHandle nh_local_;
	_InterruptCallback interrupt_callback_;
	__DataType * data_;
	std::thread * m_thread_;

	volatile bool running_;

public:
	Nodelet()
	:
		nodelet::Nodelet(), data_( NULL ), m_thread_( NULL ), running_( false )
	{
		PRINT_INFO( "--------------------" );
		PRINT_INFO( "Setting up nodelet..." );
		PRINT_INFO( "Done setting up nodelet" );
	}

	virtual ~Nodelet()
	{
		PRINT_INFO( "--------------------" );
		PRINT_INFO( "Shutting down nodelet..." );
		if ( running_ && m_thread_ )
		{
			PRINT_INFO( "Interrupting blocking processes..." );
			if( interrupt_callback_ )
			{
				interrupt_callback_();
				PRINT_INFO( "Done interrupting blocking processes" );
			}
			else PRINT_INFO( "No interrupt callback registered" );

			PRINT_INFO( "Stopping thread..." );
			if( m_thread_ ) m_thread_->join();
			PRINT_INFO( "Done stopping thread" );

			PRINT_INFO( "Deleting data..." );
			if( data_ ) delete data_;
			PRINT_INFO( "Done deleting data" );
		}
		if( m_thread_ ) delete m_thread_;
		PRINT_INFO( "Done shutting down nodelet" );
		PRINT_INFO( "--------------------\n" );
	}

	template<class __BaseCallerType, class __CallerType>
	void registerInterrupt( void(__BaseCallerType::*callback)(), __CallerType* caller )
	{
		interrupt_callback_ = std::bind( callback, caller );
	}
	
	void registerInerrupt( _InterruptCallback callback )
	{
		interrupt_callback_ = callback;
	}

	void onInit()
	{
		PRINT_INFO( "Initializing..." );

		PRINT_INFO( "Constructing data..." );
		constructData();
		PRINT_INFO( "Done constructing data" );


		PRINT_INFO( "Starting thread..." );
		running_ = true;
		m_thread_ = new std::thread( &Nodelet::spin,
		                             this );
		PRINT_INFO( "Done starting thread" );

		PRINT_INFO( "Done initializing" );
	}

	virtual void constructData()
	{
		data_ = new __DataType( getPrivateNodeHandle() );
		registerInterrupt( &__DataType::interrupt, data_ );
	}

	virtual void spin()
	{
		data_->spin();
	}
};

}

#endif // BASE_LIBS_BASE_LIBS_NODELET_H_
