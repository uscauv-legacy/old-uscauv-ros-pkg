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
