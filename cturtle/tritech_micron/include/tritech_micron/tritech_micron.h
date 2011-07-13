/*******************************************************************************
 *
 *      tritech_micron
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
 *      * Neither the name of "tritech_micron-RelWithDebInfo@tritech_micron" nor the names of its
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

#ifndef TRITECH_MICRON_H_
#define TRITECH_MICRON_H_

#include <base_node/base_node.h>
#include <common_utils/math.h>
#include <tritech_micron/ScanLine.h>
#include <tritech_micron/TritechMicronConfig.h>
#include <tritech_micron/tritech_micron_driver.h>
#include <iostream>

typedef unsigned int _DimType;

typedef tritech_micron::TritechMicronConfig _ReconfigureType;
typedef BaseNode<_ReconfigureType> _BaseNode;
typedef tritech_micron::ScanLine _ScanLineMsgType;
typedef tritech_micron::IntensityBin _IntensityBinMsgType;

typedef float _StepType;
typedef float _AngleType;
typedef std::vector<unsigned char> _IntensityBinsRawType;

class TritechMicron: public _BaseNode
{
public:
	ros::Publisher scan_line_pub_;

	/* params */
	bool simulate_;
	std::string frame_id_;
	std::string port_;int num_bins_;
	double range_;
	double velocity_of_sound_;

	/* constructor params */
	_AngleType scan_angle_;
	ros::Time last_time_;

	TritechMicronDriver * driver_;

	TritechMicron( ros::NodeHandle & nh ) :
			_BaseNode( nh ), scan_angle_( 0 ), last_time_( ros::Time::now() ), driver_( NULL )
	{
		scan_line_pub_ = nh_local_.advertise<_ScanLineMsgType>( "scan_line",
		                                                        1 );
		nh_local_.param( "simulate",
		                 simulate_,
		                 false );
		bool use_debug_mode;
		nh_local_.param( "use_debug_mode",
		                 use_debug_mode,
		                 false );
		nh_local_.param( "frame_id",
		                 frame_id_,
		                 std::string( "/tritech_micron" ) );
		nh_local_.param( "port",
		                 port_,
		                 std::string( "/dev/ttyUSB0" ) );
		nh_local_.param( "num_bins",
		                 num_bins_,
		                 200 );
		nh_local_.param( "range",
		                 range_,
		                 10.0 );
		nh_local_.param( "velocity_of_sound",
		                 velocity_of_sound_,
		                 1500.0 );

		std::string angle_step_size_name;
		nh_local_.param( "angle_step_size",
		                 angle_step_size_name,
		                 std::string( "medium" ) );
		tritech::mtHeadCommandMsg::stepAngleSize_t angle_step_size = getStepAngleSize( angle_step_size_name );

		if ( !simulate_ )
		{
			driver_ = new TritechMicronDriver( use_debug_mode );

			driver_->registerScanLineCallback( std::bind( &TritechMicron::publish,
			                                              this,
			                                              std::placeholders::_1,
			                                              std::placeholders::_2,
			                                              std::placeholders::_3 ) );
			if ( !driver_->connect( port_.c_str(),
			                        num_bins_,
			                        range_,
			                        velocity_of_sound_,
			                        angle_step_size ) )
			{
				ROS_ERROR( "Could not connect to device; simulating instead." );
				simulate_ = true;
			}
		}
	}

	~TritechMicron()
	{

		if ( driver_ )
		{
			ROS_INFO( "Disconnecting Sonar!" );
			driver_->disconnect();
			delete driver_;
		}
	}

	static tritech::mtHeadCommandMsg::stepAngleSize_t getStepAngleSize( const std::string & name )
	{
		if ( name == "crazy_low" ) return tritech::mtHeadCommandMsg::stepAngleSize_t::CrazyLow;
		if ( name == "very_low" ) return tritech::mtHeadCommandMsg::stepAngleSize_t::VeryLow;
		if ( name == "low" ) return tritech::mtHeadCommandMsg::stepAngleSize_t::Low;
		if ( name == "medium" ) return tritech::mtHeadCommandMsg::stepAngleSize_t::Medium;
		if ( name == "high" ) return tritech::mtHeadCommandMsg::stepAngleSize_t::High;
		if ( name == "ultimate" ) return tritech::mtHeadCommandMsg::stepAngleSize_t::Ultimate;

		return tritech::mtHeadCommandMsg::stepAngleSize_t::Low;
	}

	void publish( _AngleType scan_angle,
	              _StepType bin_distance_step,
	              _IntensityBinsRawType & intensity_bins )
	{
		_ScanLineMsgType::Ptr scan_line_msg( new _ScanLineMsgType );
		scan_line_msg->header.stamp = ros::Time::now();
		scan_line_msg->header.frame_id = frame_id_;
		scan_line_msg->angle = scan_angle - 180.0;
		scan_line_msg->bin_distance_step = bin_distance_step;

		scan_line_msg->bins.reserve( intensity_bins.size() );

		for ( _DimType i = 0; i < intensity_bins.size(); ++i )
		{
			_IntensityBinMsgType bin;
			bin.distance = bin_distance_step * ( i + 1 );
			bin.intensity = intensity_bins[i];
			scan_line_msg->bins.push_back( bin );
		}

		scan_line_pub_.publish( scan_line_msg );
	}

	void spinOnce()
	{
		if ( !simulate_ ) return;

		ros::Time now = ros::Time::now();

		_IntensityBinsRawType intensity_bins( reconfigure_params_.num_bins );
		for ( _DimType i = 0; i < reconfigure_params_.num_bins; ++i )
		{
			intensity_bins[i] = reconfigure_params_.intensity
			        * math_utils::normalizedGaussian( reconfigure_params_.bin_distance_step * ( i + 1 ) - reconfigure_params_.distance,
			                                          reconfigure_params_.intensity_variance );
		}

		publish( scan_angle_,
		         reconfigure_params_.bin_distance_step,
		         intensity_bins );

		if ( reconfigure_params_.use_manual_angle ) scan_angle_ = reconfigure_params_.manual_angle;
		else scan_angle_ += reconfigure_params_.scan_angle_velocity * ( now - last_time_ ).toSec();

		scan_angle_ = scan_angle_ > 180.0 ? scan_angle_ - 360.0 : scan_angle_ < -180 ? scan_angle_ + 360 :
		                                                                               scan_angle_;

		last_time_ = now;
	}

};

#endif /* TRITECH_MICRON_H_ */
