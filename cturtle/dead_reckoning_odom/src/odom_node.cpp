/*******************************************************************************
 *
 *      odom_node
 *
 *      Copyright (c) 2010,
 *
 *      Edward T. Kaszubski (ekaszubski@gmail.com),
 *      Michael Montalbo (mmontalbo@gmail.com)
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

#include <base_tf_tranceiver/base_tf_tranceiver.h>
#include <seabee3_driver_base/MotorCntl.h>
#include <seabee3_driver_base/Depth.h>
#include <xsens_node/Imu.h>
#include <seabee3_common/movement_common.h>
#include <seabee3_common/SetDesiredPose.h>
#include <seabee3_common/Vector3Masked.h>
#include <std_srvs/Empty.h>
#include <math.h>

#define POS_TRANSLATE_COEFF   0.004
#define NEG_TRANSLATE_COEFF   0.001
#define POS_STRAFE_COEFF      0.002
#define NEG_STRAFE_COEFF      0.002

class OdomNode: public BaseTfTranceiver<>
{
private:
	ros::Time last_update_time_;

	double depth_;

	geometry_msgs::Twist change_in_pose_, last_pose_;
	geometry_msgs::Vector3 pos_;

	ros::Subscriber motor_cntl_sub_, imu_sub_, depth_sub_;

	tf::Transform current_pose_tf_;

	geometry_msgs::Twist current_pose_;

	std::string global_frame_;

public:
	OdomNode( ros::NodeHandle & nh ) :
		BaseTfTranceiver<>( nh )
	{
		last_update_time_ = ros::Time( -1 );

		nh_local_.param( "global_frame", global_frame_, std::string( "/landmark_map" ) );

		motor_cntl_sub_ = nh_local_.subscribe( "motor_cntl", 1, &OdomNode::motorCntlCB, this );
		imu_sub_ = nh_local_.subscribe( "/xsens/custom_data", 1, &OdomNode::imuCB, this );
		depth_sub_ = nh_local_.subscribe( "/seabee3/depth", 1, &OdomNode::depthCB, this );
	}

	float getSimpleSpeed( int pwr, int translate )
	{
		if ( pwr > 0.0 && translate ) return pwr * POS_TRANSLATE_COEFF;
		else if ( pwr <= 0.0 && translate ) return pwr * NEG_TRANSLATE_COEFF;
		else if ( pwr > 0.0 && !translate ) return pwr * POS_STRAFE_COEFF;
		else if ( pwr <= 0.0 && !translate ) return pwr * NEG_STRAFE_COEFF;
		else return 0.0;
	}

	void imuCB( const xsens_node::ImuConstPtr & msg )
	{
		current_pose_.angular.x = msg->ori.x;
		current_pose_.angular.y = msg->ori.y;
		current_pose_.angular.z = msg->ori.z;
	}

	void depthCB( const seabee3_driver_base::DepthConstPtr & msg )
	{
		current_pose_.linear.z = -msg->value;
	}

	void motorCntlCB( const seabee3_driver_base::MotorCntlConstPtr & msg )
	{
		/*ROS_INFO( "%d %d %d %d %d %d %d %d %d", msg->motors[0], msg->motors[1], msg->motors[2], msg->motors[3], msg->motors[4], msg->motors[5], msg->motors[6], msg->motors[7], msg->motors[8] );
		 //ROS_INFO("motorCntlCallback");
		 //Create some persistent variables for this callback
		 static double th = 0.0;
		 static double last_depth = 0.0;
		 static double last_th = 0.0;
		 static ros::Time last_time = ros::Time::now();
		 ros::Time current_time = ros::Time::now();


		 //Find the amount of elapsed time
		 double dt = ( current_time - last_time ).toSec();
		 ROS_INFO( "dt %f", dt );


		 //Get the orientation from the imu
		 th = ori.z * M_PI / 180.0;


		 //Get the average forward speed
		 int right_thrust_speed = msg->motors[FWD_RIGHT_THRUSTER];
		 int left_thrust_speed = msg->motors[FWD_LEFT_THRUSTER];

		 double trans_speed = getSimpleSpeed( right_thrust_speed, 1 );


		 //Compute our predicted velocity from the motor commands
		 double vx = cos( th ) * trans_speed;
		 double vy = sin( th ) * trans_speed;


		 // if we are strafing, add that component to our velocities
		 ROS_INFO( "lin y %f", linear.y );
		 if ( linear.y != 0.0 )
		 {
		 int front_strafe_speed = msg->motors[STRAFE_FRONT_THRUSTER];
		 int back_strafe_speed = msg->motors[STRAFE_BACK_THRUSTER];

		 double strafe_speed = getSimpleSpeed( back_strafe_speed, 0 );
		 ROS_INFO( "strafe speed %f; %d %d", strafe_speed, front_strafe_speed, back_strafe_speed );
		 vx += cos( th + ( M_PI / 2 ) ) * strafe_speed;
		 vy += sin( th + ( M_PI / 2 ) ) * strafe_speed;
		 }

		 double vth = ( th - last_th ) / dt;
		 double vz = ( depth_ - last_depth ) / dt;


		 //compute the change in planar position from the speed and the elapsed time
		 double delta_x = vx * dt;
		 double delta_y = vy * dt;


		 //Integrate!
		 x += delta_x;
		 y += delta_y;
		 z = depth_;

		 odom_trans = tf::Transform( tf::Quaternion( ori.z, ori.y, ori.x ), tf::Vector3( x, y, z ) );


		 //Update our last_* variables
		 last_time = current_time;
		 last_depth = depth_;
		 last_th = th;*/
	}

	virtual void spinOnce()
	{
		if ( last_update_time_ != ros::Time( -1 ) )
		{
			ros::Duration dt = ros::Time::now() - last_update_time_;

			const double t1 = dt.toSec();
			current_pose_ >> current_pose_tf_;

			publishTfFrame( current_pose_tf_, global_frame_, "/seabee3/base_link" );
		}

		last_update_time_ = ros::Time::now();
	}
};

int main( int argc, char** argv )
{
	ros::init( argc, argv, "dead_reckoning_odom" );
	ros::NodeHandle nh( "~" );

	OdomNode odom_node( nh );
	odom_node.spin();

	return 0;
}
