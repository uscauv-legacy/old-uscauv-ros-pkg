/*******************************************************************************
 *
 *      xsens_node
 * 
 *      Copyright (c) 2010,
 *
 *      Edward T. Kaszubski (ekaszubski@gmail.com)
 *
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
 *      * Neither the name of the USC nor the names of its
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
#include <base_node/base_node.h>
#include <queue> // for queue
#include <tf/tf.h> // for tf::Vector3
#include <xsens/XSensDriver.h> // for XSensDriver
#include <geometry_msgs/Vector3.h>
#include <math.h> //for pow, sqrt
#include <common_utils/math.h>
//msgs
#include <sensor_msgs/Imu.h> // for outgoing IMU data;
#include <xsens_node/Imu.h> // for backwards-compatibility; also gives euler angles
#include <std_msgs/Bool.h> //for Bool
//srvs
#include <xsens_node/CalibrateRPY.h> // for CalibrateRPY
#include <std_srvs/Empty.h> //for Empty

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

class XSensNode: public BaseNode<>
{
private:
	std::queue<tf::Vector3> ori_data_cache_;

	// offset from imu's "north"
	tf::Vector3 ori_comp_;
	//
	tf::Vector3 drift_comp_;
	tf::Vector3 drift_comp_total_;

	std::string port_, frame_id_;
	bool drift_calibrated_, autocalibrate_, assume_calibrated_;
	double orientation_stdev_, angular_velocity_stdev_, linear_acceleration_stdev_, max_drift_rate_;

	int drift_calibration_steps_, ori_calibration_steps_;

	ros::NodeHandle nh_local_;
	ros::Publisher imu_pub_;
	ros::Publisher custom_imu_pub_;
	ros::Publisher is_calibrated_pub_;
	ros::ServiceServer calibrate_rpy_drift_srv_;
	ros::ServiceServer calibrate_rpy_ori_srv_;

	XSensDriver * imu_driver_;
	const static unsigned int IMU_DATA_CACHE_SIZE = 2;

public:
	XSensNode( ros::NodeHandle & nh ) :
		BaseNode<> ( nh )
	{
		nh_local_.param( "port", port_, std::string( "/dev/ttyUSB0" ) );
		nh_local_.param( "frame_id", frame_id_, std::string( "imu" ) );
		nh_local_.param( "autocalibrate", autocalibrate_, true );
		nh_local_.param( "orientation_stdev", orientation_stdev_, 0.035 );
		nh_local_.param( "angular_velocity_stdev", angular_velocity_stdev_, 0.012 );
		nh_local_.param( "linear_acceleration_stdev", linear_acceleration_stdev_, 0.098 );
		nh_local_.param( "max_drift_rate", max_drift_rate_, 0.001 );
		nh_local_.param( "assume_calibrated", assume_calibrated_, false );
		nh_local_.param( "drift_calibration_steps", drift_calibration_steps_, 550 );
		nh_local_.param( "ori_calibration_steps", ori_calibration_steps_, 110 );

		drift_calibrated_ = true;

		imu_pub_ = nh.advertise<sensor_msgs::Imu> ( "data", 1 );
		custom_imu_pub_ = nh.advertise<xsens_node::Imu> ( "/xsens/custom_data", 1 );
		is_calibrated_pub_ = nh.advertise<std_msgs::Bool> ( "is_calibrated", 1 );
		calibrate_rpy_drift_srv_ = nh.advertiseService( "calibrate_rpy_drift", &XSensNode::calibrateRPYDriftCB, this );
		calibrate_rpy_ori_srv_ = nh.advertiseService( "calibrate_rpy_ori", &XSensNode::calibrateRPYOriCB, this );

		imu_driver_ = new XSensDriver();
		if ( !imu_driver_->initMe() )
		{
			ROS_FATAL( "Failed to connect to IMU. Exiting..." );
			_Exit( 1 );
		}
	}

	virtual ~XSensNode()
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
			imu_driver_->ori_ >> temp;

			while ( ori_data_cache_.size() >= IMU_DATA_CACHE_SIZE )
			{
				ori_data_cache_.pop();
			}

			ori_data_cache_.push( temp );
		}
	}

	void runFullCalibration()
	{
		// compensate for drift first, then zero out the angle
		runRPYDriftCalibration();
		checkCalibration();

		runRPYOriCalibration();
	}

	void checkCalibration()
	{
		ROS_INFO( "Checking calibration..." );

		double drift_rate = sqrt( pow( drift_comp_.x(), 2 ) + pow( drift_comp_.y(), 2 ) + pow( drift_comp_.z(), 2 ) );
		drift_calibrated_ = drift_rate <= max_drift_rate_;

		ROS_INFO( "Drift rate: %f Max drift rate: %f", drift_rate, max_drift_rate_ );

		std_msgs::Bool is_calibrated_msg;
		is_calibrated_msg.data = drift_calibrated_;
		is_calibrated_pub_.publish( is_calibrated_msg );
	}

	void runRPYOriCalibration()
	{
		runRPYOriCalibration( (uint) ori_calibration_steps_ );
	}

	// assuming the robot is not moving, calculate the IMU's "north" and offset all future measurements by this amount
	void runRPYOriCalibration( uint n )
	{
		ROS_INFO( "Running ori calibration..." );
		//reset the vector to <0, 0, 0>
		ori_comp_ *= 0.0;
		for ( int i = 0; i < n && ros::ok(); i++ )
		{
			updateIMUData();
			ori_comp_ += ori_data_cache_.front();
			ros::spinOnce();
			ros::Rate( 110 ).sleep();
		}

		ori_comp_ /= (double) ( -n );
	}

	void runRPYDriftCalibration()
	{
		runRPYDriftCalibration( (uint) drift_calibration_steps_ );
	}

	// see how far the IMU drifts in the given time and try to compensate
	void runRPYDriftCalibration( uint n )
	{
		ROS_INFO( "Running drift calibration..." );
		//reset the vector to <0, 0, 0>
		drift_comp_ *= 0.0;
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

	// entry point for service
	bool calibrateRPYOriCB( xsens_node::CalibrateRPY::Request &req, xsens_node::CalibrateRPY::Response &res )
	{
		runRPYOriCalibration( req.num_samples );

		ori_comp_ >> res.calibration;

		return true;
	}

	// entry point for service
	bool calibrateRPYDriftCB( xsens_node::CalibrateRPY::Request &req, xsens_node::CalibrateRPY::Response &res )
	{
		runRPYDriftCalibration( req.num_samples );

		drift_comp_ >> res.calibration;

		checkCalibration();

		return true;
	}

	// entry point for service
	bool calibrateCB( std_srvs::Empty::Request & req, std_srvs::Empty::Response & res )
	{
		runFullCalibration();
		return true;
	}

	virtual void spinOnce()
	{
		if ( autocalibrate_ && !drift_calibrated_ ) runRPYDriftCalibration();

		sensor_msgs::Imu imu_msg;
		xsens_node::Imu custom_imu_msg;

		updateIMUData();

		imu_driver_->accel_ >> imu_msg.linear_acceleration;
		imu_driver_->gyro_ >> imu_msg.angular_velocity;

		imu_driver_->accel_ >> custom_imu_msg.accel;
		imu_driver_->gyro_ >> custom_imu_msg.gyro;
		imu_driver_->mag_ >> custom_imu_msg.mag;

		// drift_comp_total_ += drift_comp_;

		imu_driver_->ori_ += drift_comp_total_;

		tf::Vector3 temp;
		imu_driver_->ori_ >> temp;
		temp += ori_comp_;

		temp >> custom_imu_msg.ori;

		custom_imu_msg.ori.x = math_utils::degToRad( custom_imu_msg.ori.x );
		custom_imu_msg.ori.y = math_utils::degToRad( custom_imu_msg.ori.y );
		custom_imu_msg.ori.z = math_utils::degToRad( custom_imu_msg.ori.z );

		tf::Quaternion ori( temp.z(), temp.y(), temp.x() );

		imu_msg.orientation.w = ori.w();
		imu_msg.orientation.x = ori.x();
		imu_msg.orientation.y = ori.y();
		imu_msg.orientation.z = ori.z();

		imu_msg.angular_velocity_covariance[0] = imu_msg.angular_velocity_covariance[4] = imu_msg.angular_velocity_covariance[8] = angular_velocity_stdev_ * angular_velocity_stdev_;
		imu_msg.linear_acceleration_covariance[0] = imu_msg.linear_acceleration_covariance[4] = imu_msg.linear_acceleration_covariance[8] = linear_acceleration_stdev_ * linear_acceleration_stdev_;
		imu_msg.orientation_covariance[0] = imu_msg.orientation_covariance[4] = imu_msg.orientation_covariance[8] = orientation_stdev_ * orientation_stdev_;

		imu_pub_.publish( imu_msg );
		custom_imu_pub_.publish( custom_imu_msg );
		//imu_pub_raw_.publish( msg_raw );
	}
};

int main( int argc, char** argv )
{
	ros::init( argc, argv, "xsens_node" );
	ros::NodeHandle nh ( "~" );

	XSensNode xsens_node( nh );
	xsens_node.spin();
	
	return 0;
}
