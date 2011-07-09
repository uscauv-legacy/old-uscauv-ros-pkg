/*******************************************************************************
 *
 *      scan_line_converter
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

#ifndef SCAN_LINE_CONVERTER_H_
#define SCAN_LINE_CONVERTER_H_

#include <base_node/base_node.h>
#include <common_utils/math.h>
#include <tritech_micron/ScanLine.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <tritech_micron/ScanLineConverterConfig.h>

typedef unsigned int _DimType;

typedef tritech_micron::ScanLineConverterConfig _ReconfigureType;
typedef BaseNode<_ReconfigureType> _BaseNode;
typedef tritech_micron::ScanLine _ScanLineMsgType;
typedef tritech_micron::IntensityBin _IntensityBinMsgType;
typedef sensor_msgs::LaserScan _LaserScanMsgType;
typedef sensor_msgs::PointCloud _PointCloudMsgType;

typedef float _StepType;
typedef float _AngleType;
typedef std::vector<unsigned char> _IntensityBinsRawType;

class ScanLineConverter: public _BaseNode
{
public:
	ros::Subscriber scan_line_sub_;
	ros::Publisher laser_scan_pub_;
	ros::Publisher point_cloud_pub_;

	ScanLineConverter( ros::NodeHandle & nh ) :
			_BaseNode( nh )
	{
		scan_line_sub_ = nh_local_.subscribe( "scan_line",
		                                      1,
		                                      &ScanLineConverter::scanLineCB,
		                                      this );

		laser_scan_pub_ = nh_local_.advertise<_LaserScanMsgType>( "laser_scan",
		                                                          1 );

		point_cloud_pub_ = nh_local_.advertise<_PointCloudMsgType>( "point_cloud",
		                                                           1 );
		initCfgParams();
	}

	void scanLineCB( const _ScanLineMsgType::ConstPtr & scan_line_msg )
	{
		publishLaserScan( scan_line_msg );
		publishPointCloud( scan_line_msg );
	}

	void publishLaserScan( const _ScanLineMsgType::ConstPtr & scan_line_msg )
	{
		_LaserScanMsgType::Ptr laser_scan_msg( new _LaserScanMsgType );
		laser_scan_msg->header = scan_line_msg->header;
		laser_scan_msg->angle_increment = math_utils::degToRad( 0 );
		laser_scan_msg->angle_min = math_utils::degToRad( scan_line_msg->angle );
		laser_scan_msg->angle_max = math_utils::degToRad( scan_line_msg->angle );
		laser_scan_msg->range_min = scan_line_msg->bins.front().distance - 0.1;
		laser_scan_msg->range_max = scan_line_msg->bins.back().distance + 0.1;
		//laser_scan_msg->time_increment = 0.1;
		//laser_scan_msg->scan_time = 0.1;

		for ( _DimType i = 0; i < scan_line_msg->bins.size(); ++i )
		{
			if ( !reconfigure_params_.use_laser_threshold || scan_line_msg->bins[i].intensity >= reconfigure_params_.min_laser_intensity_threshold )
			{
				laser_scan_msg->intensities.push_back( scan_line_msg->bins[i].intensity / 255.0 );
				laser_scan_msg->ranges.push_back( scan_line_msg->bins[i].distance );
			}

			if ( reconfigure_params_.use_laser_threshold && scan_line_msg->bins[i].intensity >= reconfigure_params_.min_laser_intensity_threshold )
			{
				// for visualization, at least two laser scans must be sent out...for whatever reason
				laser_scan_msg->intensities.push_back( scan_line_msg->bins[i].intensity / 255.0 );
				laser_scan_msg->ranges.push_back( scan_line_msg->bins[i].distance );
				break;
			}
		}

		if ( laser_scan_msg->ranges.size() > 0 ) laser_scan_pub_.publish( laser_scan_msg );
	}

	void publishPointCloud( const _ScanLineMsgType::ConstPtr & scan_line_msg )
	{
		_PointCloudMsgType::Ptr point_cloud_msg( new _PointCloudMsgType );
		point_cloud_msg->header = scan_line_msg->header;

		point_cloud_msg->points.reserve( scan_line_msg->bins.size() );

		sensor_msgs::ChannelFloat32 channel;
		channel.name = "intensity";
		channel.values.reserve( scan_line_msg->bins.size() );

		for ( _DimType i = 0; i < scan_line_msg->bins.size(); ++i )
		{
			if( !reconfigure_params_.use_point_cloud_threshold || scan_line_msg->bins[i].intensity >= reconfigure_params_.min_point_cloud_intensity_threshold )
			{
				geometry_msgs::Point32 point;
				point.x = scan_line_msg->bins[i].distance * cos( math_utils::degToRad( scan_line_msg->angle ) );
				point.y = scan_line_msg->bins[i].distance * sin( math_utils::degToRad( scan_line_msg->angle ) );
				point.z = 0;
				point_cloud_msg->points.push_back( point );

				channel.values.push_back( scan_line_msg->bins[i].intensity / 255.0 );
			}
		}

		point_cloud_msg->channels.push_back( channel );

		point_cloud_pub_.publish( point_cloud_msg );
	}
};

#endif /* SCAN_LINE_CONVERTER_H_ */
