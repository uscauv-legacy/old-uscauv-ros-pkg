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

#include <quickdev/node.h>

// policies
#include <quickdev/tf_tranceiver_policy.h>
#include <quickdev/service_server_policy.h>

// objects
#include <quickdev/multi_publisher.h>
#include <queue> // for queue
#include <tf/tf.h> // for tf::Vector3
#include <xsens_driver/xsens_driver.h> // for XSensDriver

// utils
#include <math.h> //for pow, sqrt
#include <quickdev/unit_conversions.h> // for unit conversions, specifically radian <-> degree
#include <quickdev/message_conversions.h> // for message conversions, specifically tf::Vector3 <-> geometry_msgs::Vector3

//msgs
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/Imu.h> // for outgoing IMU data;
#include <seabee3_msgs/Imu.h> // for backwards-compatibility; also gives euler angles
#include <std_msgs/Bool.h> //for Bool

//srvs
#include <seabee3_msgs/CalibrateRPY.h> // for CalibrateRPY
#include <std_srvs/Empty.h> //for Empty

typedef sensor_msgs::Imu _ImuMsg;
typedef seabee3_msgs::Imu _SeabeeImuMsg;
typedef std_msgs::Bool _BoolMsg;

typedef seabee3_msgs::CalibrateRPY _CalibrateRPYSrv;
typedef std_srvs::Empty _EmptySrv;

typedef quickdev::TfTranceiverPolicy _TfTranceiverPolicy;
typedef quickdev::ServiceServerPolicy<_CalibrateRPYSrv, 0> _CalibrateAmbientLinearAccelServiceServerPolicy;
typedef quickdev::ServiceServerPolicy<_CalibrateRPYSrv, 1> _CalibrateRPYOriServiceServerPolicy;


void operator +=( XSensDriver::Vector3 & v1, tf::Vector3 & v2 )
{
    v1.x += v2.getX();
    v1.y += v2.getY();
    v1.z += v2.getZ();
}

typedef XSensDriver::Vector3 _XSensVector3;

DECLARE_UNIT_CONVERSION_LAMBDA( _XSensVector3, _Vector3, xsens_vec, return _Vector3( xsens_vec.x, xsens_vec.y, xsens_vec.z ); );
DECLARE_UNIT_CONVERSION_LAMBDA( _XSensVector3, _Vector3Msg, xsens_vec, return unit::convert<_Vector3Msg>( unit::convert<_Vector3>( xsens_vec ) ); );
DECLARE_UNIT_CONVERSION_LAMBDA( _XSensVector3, _Quaternion, xsens_vec, return unit::convert<_Quaternion>( tf::Vector3( Radian( Degree( xsens_vec.x ) ), Radian( Degree( xsens_vec.y ) ), Radian( Degree( xsens_vec.z ) ) ) ); );
DECLARE_UNIT_CONVERSION_LAMBDA( _XSensVector3, _QuaternionMsg, xsens_vec, return unit::convert<_QuaternionMsg>( unit::convert<_Quaternion>( xsens_vec ) ); );

// declare a node called XsensDriverNode
// a quickdev::RunablePolicy is automatically prepended to the list of policies our node will use
// to use more policies, simply list them here:
//
// QUICKDEV_DECLARE_NODE( XsensDriver, SomePolicy1, SomePolicy2 )
//
QUICKDEV_DECLARE_NODE( XsensDriver, _TfTranceiverPolicy, _CalibrateAmbientLinearAccelServiceServerPolicy, _CalibrateRPYOriServiceServerPolicy )

// declare a class called XsensDriverNode
//
QUICKDEV_DECLARE_NODE_CLASS( XsensDriver )
{
private:
    ros::MultiPublisher<> multi_pub_;

    // offset from imu's "north"
    tf::Vector3 relative_orientation_offset_;
    tf::Vector3 rotation_compensated_ambient_linear_acceleration_;

    std::string port_, frame_id_;
    double orientation_stdev_, angular_velocity_stdev_, linear_acceleration_stdev_;

    int ambient_linear_accel_calibration_steps_, ori_calibration_steps_;

    boost::shared_ptr<XSensDriver> imu_driver_ptr_;

    QUICKDEV_DECLARE_NODE_CONSTRUCTOR( XsensDriver )
    {
        initPolicies<QUICKDEV_GET_RUNABLE_POLICY()>();
    }

    QUICKDEV_SPIN_FIRST()
    {
        QUICKDEV_GET_RUNABLE_NODEHANDLE( nh_rel );
        nh_rel.param( "port", port_, std::string( "/dev/seabee/imu" ) );
        nh_rel.param( "frame_id", frame_id_, std::string( "imu" ) );
        nh_rel.param( "orientation_stdev", orientation_stdev_, 0.035 );
        nh_rel.param( "angular_velocity_stdev", angular_velocity_stdev_, 0.012 );
        nh_rel.param( "linear_acceleration_stdev", linear_acceleration_stdev_, 0.098 );
        nh_rel.param( "ambient_linear_accel_calibration_steps", ambient_linear_accel_calibration_steps_, 550 );
        nh_rel.param( "ori_calibration_steps", ori_calibration_steps_, 110 );

        multi_pub_.addPublishers
        <
            _ImuMsg,
            _ImuMsg,
            _SeabeeImuMsg,
            _BoolMsg
        >( nh_rel,
        {
            "imu",
            "rot_comp_imu",
            "seabee_imu",
            "is_calibrated"
        } );

        _CalibrateAmbientLinearAccelServiceServerPolicy::registerCallback( quickdev::auto_bind( &XsensDriverNode::calibrateAmbientLinearAccelCB, this ) );
        _CalibrateRPYOriServiceServerPolicy::registerCallback( quickdev::auto_bind( &XsensDriverNode::calibrateRPYOriCB, this ) );

        initPolicies
        <
            _CalibrateAmbientLinearAccelServiceServerPolicy,
            _CalibrateRPYOriServiceServerPolicy
        >
        (
            "enable_key_ids", true,
            "service_name_param0", std::string( "calibrate_ambient_linear_accel" ),
            "service_name_param1", std::string( "calibrate_rpy_ori" )
        );

        imu_driver_ptr_ = boost::make_shared<XSensDriver>( port_ );
        if ( !imu_driver_ptr_->initMe() )
        {
            ROS_FATAL( "Failed to connect to IMU. Exiting..." );
            _Exit( 1 );
        }

        initPolicies<quickdev::policy::ALL>();
    }

    static tf::Vector3 toRad( tf::Vector3 const & vec )
    {
        return tf::Vector3( Radian( Degree( vec.getX() ) ), Radian( Degree( vec.getY() ) ), Radian( Degree( vec.getZ() ) ) );
    }

    static tf::Vector3 toDeg( tf::Vector3 const & vec )
    {
        return tf::Vector3( Degree( Radian( vec.getX() ) ), Degree( Radian( vec.getY() ) ), Degree( Radian( vec.getZ() ) ) );
    }

    void updateIMUData()
    {
        if ( !imu_driver_ptr_->updateData() ) ROS_WARN( "Failed to update data during this cycle..." );
    }

    void runFullCalibration()
    {
        // compensate for ambient acceleration, then zero out the angle
        runAmbientLinearAccelCalibration();
        runRPYOriCalibration();
    }
/*
    void checkCalibration()
    {
        ROS_INFO( "Checking calibration..." );

        double drift_rate = sqrt( pow( drift_comp_.x(), 2 ) + pow( drift_comp_.y(), 2 ) + pow( drift_comp_.z(), 2 ) );
        drift_calibrated_ = drift_rate <= max_drift_rate_;

        ROS_INFO( "Drift rate: %f Max drift rate: %f", drift_rate, max_drift_rate_ );

        _BoolMsg is_calibrated_msg;
        is_calibrated_msg.data = drift_calibrated_;
        multi_pub_.publish( "is_calibrated", is_calibrated_msg );
    }
*/
    void runRPYOriCalibration()
    {
        runRPYOriCalibration( (size_t) ori_calibration_steps_ );
    }

    // assuming the robot is not moving, calculate the IMU's "north" and offset all future measurements by this amount
    void runRPYOriCalibration( size_t const & num_steps )
    {
        ROS_INFO( "Running ori calibration..." );
        //reset the vector to <0, 0, 0>
        relative_orientation_offset_ *= 0.0;
        for ( size_t i = 0; i < num_steps && QUICKDEV_GET_RUNABLE_POLICY()::running(); ++i )
        {
            updateIMUData();
            relative_orientation_offset_ += toRad( unit::convert<tf::Vector3>( imu_driver_ptr_->ori_ ) );
            ros::spinOnce();
            QUICKDEV_GET_RUNABLE_POLICY()::getLoopRate()->sleep();
        }

        relative_orientation_offset_ /= double( num_steps );
    }

    void runAmbientLinearAccelCalibration()
    {
        runAmbientLinearAccelCalibration( size_t( ambient_linear_accel_calibration_steps_ ) );
    }

    //! assuming the imu is still, calculate the rotation-compensated ambient linear acceleration
    /*! This value can be subtracted from future rotation-compensated measurments to perfectly remove any ambient acceleration, ie from gravity
     */
    void runAmbientLinearAccelCalibration( size_t const & num_steps )
    {
        ROS_INFO( "Running ambient linear accel calibration..." );
        //reset the vector to <0, 0, 0>
        rotation_compensated_ambient_linear_acceleration_ *= 0.0;
        for ( size_t i = 0; i < num_steps && QUICKDEV_GET_RUNABLE_POLICY()::running(); ++i )
        {
            updateIMUData();
            rotation_compensated_ambient_linear_acceleration_ += calculateRotationCompensatedLinearAcceleration( unit::implicit_convert( imu_driver_ptr_->ori_ ), unit::implicit_convert( imu_driver_ptr_->accel_ ) );
            ros::spinOnce();
            QUICKDEV_GET_RUNABLE_POLICY()::getLoopRate()->sleep();
        }
        rotation_compensated_ambient_linear_acceleration_ /= double( num_steps ); //avg acceleration per cycle
    }

    //! Given the current orientation and linear acceleration, calculate a rotation-invariant linear acceleration value
    /*! Specifically, rotate the linear acceleration vector by the current orientation. This could be pictured as placing a virtual,
     *  non-rotatable IMU at the center of the real IMU, such that we only measure acceleration due to motion, and not due to orientation
     */
    tf::Vector3 calculateRotationCompensatedLinearAcceleration( tf::Quaternion const & orientation, tf::Vector3 const & linear_acceleration )
    {
        // create a transform from the current orientation
        tf::Transform const orientation_tf( orientation );
        // rotate the acceleration vector by that transform
        return orientation_tf * linear_acceleration;
    }

    // entry point for service
    QUICKDEV_DECLARE_SERVICE_CALLBACK( calibrateRPYOriCB, _CalibrateRPYSrv )
    {
        runRPYOriCalibration( request.num_samples );

        response.calibration = unit::implicit_convert( relative_orientation_offset_ );

        return true;
    }

    // entry point for service
    QUICKDEV_DECLARE_SERVICE_CALLBACK( calibrateAmbientLinearAccelCB, _CalibrateRPYSrv )
    {
        runAmbientLinearAccelCalibration( request.num_samples );

        response.calibration = unit::implicit_convert( rotation_compensated_ambient_linear_acceleration_ );

        return true;
    }

    // entry point for service
    QUICKDEV_DECLARE_SERVICE_CALLBACK( calibrateCB, _EmptySrv )
    {
        runFullCalibration();
        return true;
    }

    QUICKDEV_SPIN_ONCE()
    {
        updateIMUData();
        ros::Time const now = ros::Time::now();

        _ImuMsg imu_msg;
        _ImuMsg rot_comp_imu_msg;
        _SeabeeImuMsg seabee_imu_msg;

        imu_msg.header.stamp = now;
        imu_msg.header.frame_id = "/seabee3/sensors/imu";

        // our rotation vector from the IMU; convert from degrees to radians
        tf::Vector3 const orientation_rpy = toRad( unit::convert<tf::Vector3>( imu_driver_ptr_->ori_ ) );
        // our rotation vector, offset by the results of any relative orientation calibration
        tf::Vector3 const orientation_rpy_with_offset = unit::implicit_convert( orientation_rpy - relative_orientation_offset_ );
        // our orientation from the IMU
        tf::Quaternion const orientation = unit::implicit_convert( imu_driver_ptr_->ori_ );
        // our orientation from the IMU, with any offset from calibration
        tf::Quaternion const orientation_with_offset = unit::implicit_convert( orientation_rpy_with_offset );

        seabee_imu_msg.accel = unit::implicit_convert( imu_driver_ptr_->accel_ );
        seabee_imu_msg.gyro = unit::implicit_convert( imu_driver_ptr_->gyro_ );
        seabee_imu_msg.mag = unit::implicit_convert( imu_driver_ptr_->mag_ );
        seabee_imu_msg.ori = unit::implicit_convert( toDeg( orientation_rpy_with_offset ) );

        imu_msg.angular_velocity = unit::implicit_convert( imu_driver_ptr_->gyro_ );

        imu_msg.orientation = unit::implicit_convert( orientation_with_offset );

        imu_msg.angular_velocity_covariance[0] = imu_msg.angular_velocity_covariance[4] = imu_msg.angular_velocity_covariance[8] = angular_velocity_stdev_ * angular_velocity_stdev_;
        imu_msg.linear_acceleration_covariance[0] = imu_msg.linear_acceleration_covariance[4] = imu_msg.linear_acceleration_covariance[8] = linear_acceleration_stdev_ * linear_acceleration_stdev_;
        imu_msg.orientation_covariance[0] = imu_msg.orientation_covariance[4] = imu_msg.orientation_covariance[8] = orientation_stdev_ * orientation_stdev_;

        rot_comp_imu_msg = imu_msg;

        tf::Vector3 const linear_acceleration = unit::implicit_convert( imu_driver_ptr_->accel_ );
        /* get our linear acceleration, compensated by our current orientation; this will result in a linear acceleration as perceived by an IMU
         * in a fixed frame, regardless of our current orientation
        */
        tf::Vector3 const rotation_compensated_linear_acceleration = calculateRotationCompensatedLinearAcceleration( orientation, linear_acceleration );

        // remove any ambient acceleration (ie due to gravity) picked up by our calibration
        tf::Vector3 const compensated_linear_acceleration = rotation_compensated_linear_acceleration - rotation_compensated_ambient_linear_acceleration_;

        // construct a rotation-only transform from our current orientation
        tf::Transform const relative_orientation_offset_tf( orientation );

        /* rotate the above acceleration back into our current frame; this will result in the ideal rotation-compensated, ambient-acceleration-
         * compensated value that we're looking for
         */
        tf::Vector3 const rotated_compensated_linear_acceleration = relative_orientation_offset_tf.inverse() * compensated_linear_acceleration;

        imu_msg.linear_acceleration = unit::implicit_convert( imu_driver_ptr_->accel_ );
        rot_comp_imu_msg.linear_acceleration = unit::implicit_convert( rotated_compensated_linear_acceleration );

        // drift_comp_total_ += drift_comp_;
/*
        imu_driver_ptr_->ori_ += drift_comp_total_;

        tf::Vector3 temp;
        temp = unit::implicit_convert( imu_driver_ptr_->ori_ );
        temp -= relative_orientation_offset_;

        seabee_imu_msg.ori = unit::implicit_convert( temp );
*/

        _TfTranceiverPolicy::publishTransform( tf::Transform( orientation_with_offset, tf::Vector3( 0, 0, 0 ) ), "/world", "/seabee3/sensors/imu", now );
        multi_pub_.publish( "imu", imu_msg, "rot_comp_imu", rot_comp_imu_msg, "seabee_imu", seabee_imu_msg );
        //imu_pub_raw_.publish( msg_raw );
    }
};

#endif // XSENSDRIVER_XSENSDRIVER_H_
