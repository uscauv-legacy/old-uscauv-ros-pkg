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

#include <ros/ros.h>
// for dynamic reconfigure server
#include <dynamic_reconfigure/server.h>
// for cfg code
#include <base_node/../../cfg/cpp/base_libs/EmptyConfig.h>

template<typename _ReconfigureType>
struct ReconfigureSettings
{
	static bool enableReconfigure()
	{
		return true;
	}
};

template<>
struct ReconfigureSettings<base_libs::EmptyConfig>
{
	static bool enableReconfigure()
	{
		return false;
	}
};

struct BaseNodeTypes
{
	typedef base_libs::EmptyConfig _DefaultReconfigureType;
};

struct SpinModeId
{
	const static int spin = 0;
	const static int loop_spin_once = 1;
};

template<typename _ReconfigureType = BaseNodeTypes::_DefaultReconfigureType>
class BaseNode
{

protected:
	ros::NodeHandle nh_priv_;
	ros::MultiThreadedSpinner spinner_;

	dynamic_reconfigure::Server<_ReconfigureType> reconfigure_srv_;
	typename dynamic_reconfigure::Server<_ReconfigureType>::CallbackType reconfigure_callback_;

	// workaround for reconfigure ussues
	bool reconfigure_initialized_, ignore_reconfigure_;
	_ReconfigureType initial_config_params_;
	uint32_t initial_config_level_;

public:
	BaseNode( ros::NodeHandle & nh, uint threads = 3 );
	~BaseNode();
	virtual void spin( int mode = SpinModeId::spin );
	virtual void spinOnce();

protected:
	virtual void reconfigureCB( _ReconfigureType &config, uint32_t level );
	void initCfgParams();

private:
	void reconfigureCB_0( _ReconfigureType &config, uint32_t level );
};

template<typename _ReconfigureType>
BaseNode<_ReconfigureType>::BaseNode( ros::NodeHandle & nh, uint threads ) :
	nh_priv_( "~" ), spinner_( threads ), ignore_reconfigure_( !ReconfigureSettings<_ReconfigureType>::enableReconfigure() )
{
	reconfigure_initialized_ = ignore_reconfigure_;
	if ( !ignore_reconfigure_ )
	{
		reconfigure_callback_ = boost::bind( &BaseNode::reconfigureCB_0, this, _1, _2 );
		reconfigure_srv_.setCallback( reconfigure_callback_ );
	}
}

template<typename _ReconfigureType>
BaseNode<_ReconfigureType>::~BaseNode()
{
	//
}

// virtual
template<typename _ReconfigureType>
void BaseNode<_ReconfigureType>::spin( int mode )
{
	if ( mode == SpinModeId::spin ) spinner_.spin();
	else if ( mode == SpinModeId::loop_spin_once )
	{
		while ( ros::ok() )
		{
			spinOnce();
			ros::spinOnce();
		}
	}
	else spin( SpinModeId::spin );
}

// virtual
template<typename _ReconfigureType>
void BaseNode<_ReconfigureType>::spinOnce()
{
	//
}

//virtual
template<typename _ReconfigureType>
void BaseNode<_ReconfigureType>::reconfigureCB( _ReconfigureType &config, uint32_t level )
{
	ROS_DEBUG( "Reconfigure successful in base class" );
}

template<typename _ReconfigureType>
void BaseNode<_ReconfigureType>::reconfigureCB_0( _ReconfigureType &config, uint32_t level )
{
	initial_config_params_ = config;
	initial_config_level_ = level;
	reconfigureCB( config, level );
}

// virtual
template<typename _ReconfigureType>
void BaseNode<_ReconfigureType>::initCfgParams()
{
	if ( !ignore_reconfigure_ )
	{
		reconfigureCB( initial_config_params_, initial_config_level_ );
		reconfigure_initialized_ = true;
	}
}

#endif /* BASE_NODE_H_ */
