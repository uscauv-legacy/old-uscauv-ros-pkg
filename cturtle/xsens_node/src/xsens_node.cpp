/*******************************************************************************
 *
 *      xsens_node
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
 *      * Neither the name of the USC Underwater Robotics Team nor the names of its
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

//tools
#include <queue> // for queue
#include <ros/ros.h>
#include <tf/tf.h> // for tf::Vector3
#include <xsens/xsens_driver.h> // for XSensDriver
//msgs
#include <xsens_node/IMUData.h> // for outgoing IMUData
//srvs
#include <xsens_node/CalibrateRPY.h> // for CalibrateRPY

void operator +=( XSensDriver::Vector3 & v1, tf::Vector3 & v2 )
{
	v1.x += v2.getX();
	v1.y += v2.getY();
	v1.z += v2.getZ();
}

void operator >>( XSensDriver::Vector3 & v1, geometry_msgs::Vector3 & v2 )
{
	v2.x = v1.x;
	v2.y = v1.y;
	v2.z = v1.z;
}

void operator >>( XSensDriver::Vector3 & v1, tf::Vector3 & v2 )
{
	v2.setX( v1.x );
	v2.setY( v1.y );
	v2.setZ( v1.z );
}

void operator >>( tf::Vector3 & v1, geometry_msgs::Vector3 & v2 )
{
	v2.x = v1.getX();
	v2.y = v1.getY();
	v2.z = v1.getZ();
}

class XSensNode
{
private:
	std::queue<tf::Vector3> ori_data_cache_;
	tf::Vector3 ori_comp_, drift_comp_, drift_comp_total_;

	ros::NodeHandle n_priv_;
	ros::Publisher imu_pub_calib_;
	ros::Publisher imu_pub_raw_;
	ros::ServiceServer calibrate_rpy_drift_srv_;
	ros::ServiceServer calibrate_rpy_ori_srv_;

	XSensDriver * imu_driver_;
	const static unsigned int IMU_DATA_CACHE_SIZE = 2;

public:
	XSensNode( ros::NodeHandle & n ) :
		n_priv_( "~" )
	{
		imu_pub_calib_ = n.advertise<xsens_node::IMUData> ( "data_calibrated", 1 );
		imu_pub_raw_ = n.advertise<xsens_node::IMUData> ( "data_raw", 1 );
		calibrate_rpy_drift_srv_ = n.advertiseService( "calibrate_rpy_drift", &XSensNode::calibrateRPYDriftCallback, this );
		calibrate_rpy_ori_srv_ = n.advertiseService( "calibrate_rpy_ori", &XSensNode::calibrateRPYOriCallback, this );

		imu_driver_ = new XSensDriver( 0 );
		if ( !imu_driver_->initMe() )
		{
			ROS_WARN( "Failed to connect to IMU. Exiting..." );
			return;
		}

		runRPYOriCalibration( 10 );
	}

	~XSensNode()
	{
		delete imu_driver_;
	}

	void updateIMUData()
	{
		if ( !imu_driver_->updateData() )
		{
			ROS_WARN( "Failed to update data during this cycle..." );
		}
		else
		{
			tf::Vector3 temp;
			imu_driver_->ori >> temp;

			while ( ori_data_cache_.size() >= IMU_DATA_CACHE_SIZE )
			{
				ori_data_cache_.pop();
			}

			ori_data_cache_.push( temp );
		}
	}

	void runRPYOriCalibration( int n )
	{
		ori_comp_ *= 0.0; //reset the vector to <0, 0, 0>
		for ( int i = 0; i < n && ros::ok(); i++ )
		{
			updateIMUData();
			ori_comp_ += ori_data_cache_.front();
			ros::spinOnce();
			ros::Rate( 110 ).sleep();
		}

		ori_comp_ /= (double) ( -n );
	}

	void runRPYDriftCalibration( int n )
	{
		drift_comp_ *= 0.0; //reset the vector to <0, 0, 0>
		updateIMUData();
		drift_comp_ = ori_data_cache_.front();
		for ( int i = 0; i < n && ros::ok(); i++ )
		{
			updateIMUData();
			ros::spinOnce();
			ros::Rate( 110 ).sleep();
		}
		drift_comp_ -= ori_data_cache_.front();
		drift_comp_ /= (double) ( n ); //avg drift per cycle
	}

	bool calibrateRPYOriCallback( xsens_node::CalibrateRPY::Request &req, xsens_node::CalibrateRPY::Response &res )
	{
		runRPYOriCalibration( req.num_samples );

		ori_comp_ >> res.calibration;

		return true;
	}

	bool calibrateRPYDriftCallback( xsens_node::CalibrateRPY::Request &req, xsens_node::CalibrateRPY::Response &res )
	{
		runRPYDriftCalibration( req.num_samples );

		drift_comp_ >> res.calibration;

		return true;
	}

	void spin()
	{
		while ( ros::ok() )
		{
			xsens_node::IMUData msg_calib;
			xsens_node::IMUData msg_raw;

			updateIMUData();

			imu_driver_->accel >> msg_raw.accel;
			imu_driver_->gyro >> msg_raw.gyro;
			imu_driver_->mag >> msg_raw.mag;
			imu_driver_->ori >> msg_raw.ori;

			imu_driver_->accel >> msg_calib.accel;
			imu_driver_->gyro >> msg_calib.gyro;
			imu_driver_->mag >> msg_calib.mag;

			drift_comp_total_ += drift_comp_;

			imu_driver_->ori += drift_comp_total_;

			tf::Vector3 temp;
			imu_driver_->ori >> temp;
			temp += ori_comp_;

			temp >> msg_calib.ori;

			imu_pub_calib_.publish( msg_calib );
			imu_pub_raw_.publish( msg_raw );
			ros::spinOnce();
			ros::Rate( 110 ).sleep();
		}
	}
};

int main( int argc, char** argv )
{
	ros::init( argc, argv, "xsens_node" );
	ros::NodeHandle n;

	XSensNode xsens_node( n );
	xsens_node.spin();
	
	return 0;
}
