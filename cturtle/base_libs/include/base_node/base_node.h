/*******************************************************************************
 *
 *      base_node
 * 
 *      Copyright (c) 2010, Edward T. Kaszubski (ekaszubski@gmail.com)
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
 *      * Neither the name of "seabee3-ros-pkg" nor the names of its
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

#ifndef BASE_NODE_H_
#define BASE_NODE_H_

/* ROS */
#include <ros/ros.h>
// for dynamic reconfigure server
#include <dynamic_reconfigure/server.h>
// for cfg code
#include <base_node/../../cfg/cpp/base_libs/EmptyConfig.h>

// returns true if we're using a custom reconfigure type
template<typename _BaseReconfigureType>
struct ReconfigureSettings
{
	const static bool reconfigure_enabled = true;
};

// returns false if we're not using a custom reconfigure type
template<>
struct ReconfigureSettings<base_libs::EmptyConfig>
{
	const static bool reconfigure_enabled = false;
};

// define the default reconfigure type
struct BaseNodeTypes
{
	typedef base_libs::EmptyConfig _DefaultReconfigureType;
};

// our node can either be run via ros::spin() or by doing some processing and then running ros::spinOnce()
struct SpinModeId
{
	const static unsigned int SPIN = 0;
	const static unsigned int LOOP_SPIN_ONCE = 1;
};

template<typename _BaseReconfigureType = BaseNodeTypes::_DefaultReconfigureType>
class BaseNode
{
public:
	typedef _BaseReconfigureType _ReconfigureType;
	typedef dynamic_reconfigure::Server<_BaseReconfigureType> _DynamicReconfigureServerType;

protected:

	/* dynamic reconfigure */
	typename dynamic_reconfigure::Server<_BaseReconfigureType>::CallbackType reconfigure_callback_;
	bool reconfigure_initialized_;
	_BaseReconfigureType initial_config_params_;
	uint32_t initial_config_level_;

	/* constructor params */
	ros::NodeHandle nh_local_;
	ros::AsyncSpinner spinner_;
	ros::Rate * loop_rate_;
	_DynamicReconfigureServerType * reconfigure_srv_;
	bool ignore_reconfigure_;
	bool running_;

public:
	BaseNode( ros::NodeHandle & nh, std::string reconfigure_ns = "reconfigure", uint threads = 3 ) :
		nh_local_( nh ), spinner_( threads ), loop_rate_( NULL ), reconfigure_srv_( NULL ), ignore_reconfigure_( !ReconfigureSettings<_BaseReconfigureType>::reconfigure_enabled ), running_( false )
	{
		ROS_INFO( "Setting up base node..." );
		reconfigure_initialized_ = ignore_reconfigure_;
		if ( !ignore_reconfigure_ )
		{
			ROS_INFO( "Setting up reconfigure server..." );
			reconfigure_srv_ = new _DynamicReconfigureServerType( ros::NodeHandle ( nh_local_, reconfigure_ns ) );
			reconfigure_callback_ = boost::bind( &BaseNode::reconfigureCB_0, this, _1, _2 );
			reconfigure_srv_->setCallback( reconfigure_callback_ );
			ROS_INFO( "Done setting up reconfigure server" );
		}
		ROS_INFO( "Done setting up base node" );
	}

	virtual ~BaseNode()
	{
		ROS_INFO( "Shutting down base_node..." );
		if( running_ )
		{
			interrupt();
		}
		ROS_INFO( "Done shutting down base_node" );
	}

	virtual void spin( unsigned int mode = SpinModeId::SPIN, float frequency = 10.0 )
	{
		ROS_INFO( "Spinner starting up at %f Hz with mode %u...", frequency, mode );

		running_ = true;
		if( loop_rate_ ) delete loop_rate_;
		loop_rate_ = new ros::Rate( frequency );

		if ( mode == SpinModeId::SPIN )
		{
			spinner_.start();
		}
		else if ( mode == SpinModeId::LOOP_SPIN_ONCE )
		{
			//
		}
		else ROS_WARN( "Invalid spin mode selected." );

		while ( ros::ok() && running_ )
		{
			spinOnce();
			ros::spinOnce();
			loop_rate_->sleep();
		}
	}

	virtual void spinOnce()
	{
		//
	}

	virtual void interrupt()
	{
		ROS_INFO( "Interrupting spinner..." );
		running_ = false;
		spinner_.stop();
		ROS_INFO( "Done interrupting spinner" );
	}

protected:
	virtual void reconfigureCB( _BaseReconfigureType &config, uint32_t level )
	{
		ROS_DEBUG( "Reconfigure successful in base class" );
	}

	void initCfgParams()
	{
		if ( !ignore_reconfigure_ )
		{
			reconfigureCB_0( initial_config_params_, initial_config_level_ );
		}
	}

private:
	void reconfigureCB_0( _BaseReconfigureType &config, uint32_t level )
	{
		initial_config_params_ = config;
		initial_config_level_ = level;
		reconfigureCB( config, level );
		reconfigure_initialized_ = true;
	}
};

#endif /* BASE_NODE_H_ */
