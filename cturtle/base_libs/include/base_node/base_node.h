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
	static bool reconfigureEnabled()
	{
		return true;
	}
};

// returns false if we're not using a custom reconfigure type
template<>
struct ReconfigureSettings<base_libs::EmptyConfig>
{
	static bool reconfigureEnabled()
	{
		return false;
	}
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

protected:
	/* dynamic reconfigure */
	typename dynamic_reconfigure::Server<_BaseReconfigureType>::CallbackType reconfigure_callback_;
	bool reconfigure_initialized_;
	_BaseReconfigureType initial_config_params_;
	uint32_t initial_config_level_;

	/* constructor params */
	ros::NodeHandle nh_priv_;
	ros::MultiThreadedSpinner spinner_;
	dynamic_reconfigure::Server<_BaseReconfigureType> reconfigure_srv_;
	bool ignore_reconfigure_;

public:
	BaseNode( ros::NodeHandle & nh, std::string reconfigure_ns = "reconfigure", uint threads = 3 ) :
		nh_priv_( "~" ), spinner_( threads ), reconfigure_srv_( ros::NodeHandle ( nh_priv_, reconfigure_ns ) ), ignore_reconfigure_( !ReconfigureSettings<_BaseReconfigureType>::reconfigureEnabled() )
	{
		reconfigure_initialized_ = ignore_reconfigure_;
		if ( !ignore_reconfigure_ )
		{
			reconfigure_callback_ = boost::bind( &BaseNode::reconfigureCB_0, this, _1, _2 );
			reconfigure_srv_.setCallback( reconfigure_callback_ );
		}
	}

	virtual ~BaseNode()
	{
		//
	}

	virtual void spin( unsigned int mode = SpinModeId::SPIN, float frequency = 10.0 )
	{
		if ( mode == SpinModeId::SPIN ) spinner_.spin();
		else if ( mode == SpinModeId::LOOP_SPIN_ONCE )
		{
			static ros::Rate loop_rate( frequency );
			while ( ros::ok() )
			{
				spinOnce();
				ros::spinOnce();
				loop_rate.sleep();
			}
		}
		else spin( SpinModeId::SPIN );
	}

	virtual void spinOnce()
	{
		//
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
			reconfigureCB( initial_config_params_, initial_config_level_ );
			reconfigure_initialized_ = true;
		}
	}

private:
	void reconfigureCB_0( _BaseReconfigureType &config, uint32_t level )
	{
		initial_config_params_ = config;
		initial_config_level_ = level;
		reconfigureCB( config, level );
	}
};

#endif /* BASE_NODE_H_ */
