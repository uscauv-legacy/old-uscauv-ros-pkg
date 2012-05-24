/***************************************************************************
 *  include/xsens_driver/xsens_driver_node.h
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
 *  * Neither the name of usc-ros-pkg nor the names of its
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

#ifndef XSENSDRIVER_XSENSDRIVER_H_
#define XSENSDRIVER_XSENSDRIVER_H_

//tools
#include <quickdev/node.h>
#include <queue> // for queue
#include <tf/tf.h> // for tf::Vector3
#include <quickdev/tf_tranceiver_policy.h>
#include <xsens_driver/xsens_driver.h> // for XSensDriver
#include <geometry_msgs/Vector3.h>
#include <math.h> //for pow, sqrt
#include <quickdev/unit_conversions.h> // for unit conversions, specifically radian <-> degree
#include <quickdev/message_conversions.h> // for message conversions, specifically btVector3 <-> geometry_msgs::Vector3
//msgs
#include <sensor_msgs/Imu.h> // for outgoing IMU data;
#include <xsens_driver/Imu.h> // for backwards-compatibility; also gives euler angles
#include <std_msgs/Bool.h> //for Bool
//srvs
#include <xsens_driver/CalibrateRPY.h> // for CalibrateRPY
#include <std_srvs/Empty.h> //for Empty

typedef quickdev::TfTranceiverPolicy _TfTranceiverPolicy;

void operator +=( XSensDriver::Vector3 & v1, tf::Vector3 & v2 )
{
    v1.x += v2.getX();
    v1.y += v2.getY();
    v1.z += v2.getZ();
}

typedef XSensDriver::Vector3 _XSensVector3;

DECLARE_UNIT_CONVERSION_LAMBDA( _XSensVector3, _Vector3Msg, xsens_vec, _Vector3Msg msg; msg.x = xsens_vec.x; msg.y = xsens_vec.y; msg.z = xsens_vec.z; return msg; );
DECLARE_UNIT_CONVERSION_LAMBDA( _XSensVector3, _Vector3, xsens_vec, return _Vector3( xsens_vec.x, xsens_vec.y, xsens_vec.z ); );

// declare a node called XsensDriverNode
// a quickdev::RunablePolicy is automatically prepended to the list of policies our node will use
// to use more policies, simply list them here:
//
// QUICKDEV_DECLARE_NODE( XsensDriver, SomePolicy1, SomePolicy2 )
//
QUICKDEV_DECLARE_NODE( XsensDriver, _TfTranceiverPolicy )

// declare a class called XsensDriverNode
//
QUICKDEV_DECLARE_NODE_CLASS( XsensDriver )
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

    ros::Publisher imu_pub_;
    ros::Publisher custom_imu_pub_;
    ros::Publisher is_calibrated_pub_;
    ros::ServiceServer calibrate_rpy_drift_srv_;
    ros::ServiceServer calibrate_rpy_ori_srv_;

    XSensDriver * imu_driver_;
    const static unsigned int IMU_DATA_CACHE_SIZE = 2;

    // variable initializations can be appended to this constructor as a comma-separated list:
    //
    // QUICKDEV_DECLARE_NODE_CONSTRUCTOR( XsensDriver ), member1_( some_value ), member2_( some_other_value ){}
    //
    QUICKDEV_DECLARE_NODE_CONSTRUCTOR( XsensDriver )
    {
        QUICKDEV_GET_RUNABLE_NODEHANDLE( nh_rel );
        nh_rel.param( "port", port_, std::string( "/dev/seabee/imu" ) );
        nh_rel.param( "frame_id", frame_id_, std::string( "imu" ) );
        nh_rel.param( "autocalibrate", autocalibrate_, true );
        nh_rel.param( "orientation_stdev", orientation_stdev_, 0.035 );
        nh_rel.param( "angular_velocity_stdev", angular_velocity_stdev_, 0.012 );
        nh_rel.param( "linear_acceleration_stdev", linear_acceleration_stdev_, 0.098 );
        nh_rel.param( "max_drift_rate", max_drift_rate_, 0.001 );
        nh_rel.param( "assume_calibrated", assume_calibrated_, false );
        nh_rel.param( "drift_calibration_steps", drift_calibration_steps_, 550 );
        nh_rel.param( "ori_calibration_steps", ori_calibration_steps_, 110 );

        drift_calibrated_ = true;

        imu_pub_ = nh_rel.advertise<sensor_msgs::Imu> ( "data", 1 );
        custom_imu_pub_ = nh_rel.advertise<xsens_driver::Imu> ( "/xsens/custom_data", 1 );
        is_calibrated_pub_ = nh_rel.advertise<std_msgs::Bool> ( "is_calibrated", 1 );
        calibrate_rpy_drift_srv_ = nh_rel.advertiseService( "calibrate_rpy_drift", &XsensDriverNode::calibrateRPYDriftCB, this );
        calibrate_rpy_ori_srv_ = nh_rel.advertiseService( "calibrate_rpy_ori", &XsensDriverNode::calibrateRPYOriCB, this );

        imu_driver_ = new XSensDriver( port_ );
        if ( !imu_driver_->initMe() )
        {
            ROS_FATAL( "Failed to connect to IMU. Exiting..." );
            _Exit( 1 );
        }
    }

    ~XsensDriverNode()
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
            temp = unit::make_unit( imu_driver_->ori_ );

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
    bool calibrateRPYOriCB( xsens_driver::CalibrateRPY::Request &req, xsens_driver::CalibrateRPY::Response &res )
    {
        runRPYOriCalibration( req.num_samples );

        res.calibration = unit::make_unit( ori_comp_ );

        return true;
    }

    // entry point for service
    bool calibrateRPYDriftCB( xsens_driver::CalibrateRPY::Request &req, xsens_driver::CalibrateRPY::Response &res )
    {
        runRPYDriftCalibration( req.num_samples );

        res.calibration = unit::make_unit( drift_comp_ );

        checkCalibration();

        return true;
    }

    // entry point for service
    bool calibrateCB( std_srvs::Empty::Request & req, std_srvs::Empty::Response & res )
    {
        runFullCalibration();
        return true;
    }

    QUICKDEV_SPIN_FIRST()
    {
        initPolicies<quickdev::policy::ALL>();
    }

    QUICKDEV_SPIN_ONCE()
    {
        if ( autocalibrate_ && !drift_calibrated_ ) runRPYDriftCalibration();

        sensor_msgs::Imu imu_msg;
        xsens_driver::Imu custom_imu_msg;

        updateIMUData();

        imu_msg.linear_acceleration = unit::make_unit( imu_driver_->accel_ );
        imu_msg.angular_velocity = unit::make_unit( imu_driver_->gyro_ );

        custom_imu_msg.accel = unit::make_unit( imu_driver_->accel_ );
        custom_imu_msg.gyro = unit::make_unit( imu_driver_->gyro_ );
        custom_imu_msg.mag = unit::make_unit( imu_driver_->mag_ );

        // drift_comp_total_ += drift_comp_;

        imu_driver_->ori_ += drift_comp_total_;

        tf::Vector3 temp;
        temp = unit::make_unit( imu_driver_->ori_ );
        temp += ori_comp_;

        custom_imu_msg.ori = unit::make_unit( temp );

        custom_imu_msg.ori.x = Radian( Degree( custom_imu_msg.ori.x ) );
        custom_imu_msg.ori.y = Radian( Degree( custom_imu_msg.ori.y ) );
        custom_imu_msg.ori.z = Radian( Degree( custom_imu_msg.ori.z ) );

        tf::Quaternion ori( temp.z(), temp.y(), temp.x() );

        _TfTranceiverPolicy::publishTransform( btTransform( ori, btVector3( 0, 0, 0 ) ), "/world", "/seabee3/imu" );

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

#endif // XSENSDRIVER_XSENSDRIVER_H_
