#include <base_robot_driver/base_robot_driver.h>
#include <seabee3_driver_base/MotorCntl.h>
#include <seabee3_driver_base/Depth.h>
#include <xsens_node/Imu.h>
#include <seabee3_beestem/BeeStem3Driver.h>
#include <seabee3_msgs/SetDesiredPose.h>
#include <seabee3_msgs/Vector3Masked.h>
#include <std_srvs/Empty.h>

#define POS_TRANSLATE_COEFF   0.004
#define NEG_TRANSLATE_COEFF   0.001
#define POS_STRAFE_COEFF      0.002
#define NEG_STRAFE_COEFF      0.002

class OdomNode: public BaseRobotDriver
{
private:
	ros::Time last_update_time_;

	double depth_;

	geometry_msgs::Twist change_in_pose_, last_pose_;
	geometry_msgs::Vector3 pos_;

	ros::Subscriber motor_cntl_sub_, imu_sub_, depth_sub_, cmd_vel_sub_;
	ros::ServiceServer reset_pose_srv_;
	ros::ServiceServer set_desired_pose_srv_;

	tf::Transform desired_pose_tf_;
	tf::Transform current_pose_tf_;

	geometry_msgs::Twist desired_pose_;
	geometry_msgs::Twist current_pose_;

	std::string global_frame_;

public:
	OdomNode( ros::NodeHandle & nh ) :
		BaseRobotDriver( nh, "/seabee3/cmd_vel" )
	{
		last_update_time_ = ros::Time( -1 );

		nh_priv_.param( "global_frame", global_frame_, std::string( "/landmark_map" ) );

		motor_cntl_sub_ = nh.subscribe( "/seabee3/motor_cntl", 1, &OdomNode::motorCntlCB, this );
		imu_sub_ = nh.subscribe( "/xsens/data_calibrated", 1, &OdomNode::imuCB, this );
		depth_sub_ = nh.subscribe( "/seabee3/depth", 1, &OdomNode::depthCB, this );

		reset_pose_srv_ = nh.advertiseService( "/seabee3/reset_pose", &OdomNode::resetPoseCB, this );
		set_desired_pose_srv_ = nh.advertiseService( "/seabee3/set_desired_pose", &OdomNode::setDesiredPoseCB, this );
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

	}

	void depthCB( const seabee3_driver_base::DepthConstPtr & msg )
	{

	}

	void cmdVelCB( const geometry_msgs::TwistConstPtr & msg )
	{

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

	void setDesiredPoseComponent( double & mask, double & desired, double & value, double & mode )
	{
		// mode = 0 -> desired = value
		// mode = 1 -> desired += value
		if ( mask != 0.0 ) desired = value + ( mode == 1.0 ? desired : 0.0 );
	}

	void setDesiredPose( geometry_msgs::Vector3 & mask, geometry_msgs::Vector3 & desired, geometry_msgs::Vector3 & value, geometry_msgs::Vector3 & mode )
	{
		setDesiredPoseComponent( mask.x, desired.x, value.x, mode.x );
		setDesiredPoseComponent( mask.y, desired.y, value.y, mode.y );
		setDesiredPoseComponent( mask.z, desired.z, value.z, mode.z );
	}

	bool resetPoseCB( std_srvs::Empty::Request & req, std_srvs::Empty::Response & resp )
	{
		desired_pose_ = geometry_msgs::Twist();
		current_pose_ = geometry_msgs::Twist();

		return true;
	}

	bool setDesiredPoseCB( seabee3_msgs::SetDesiredPose::Request & req, seabee3_msgs::SetDesiredPose::Response & resp )
	{
		setDesiredPose( req.pos.mask, desired_pose_.linear, req.pos.values, req.pos.mode );
		setDesiredPose( req.ori.mask, desired_pose_.angular, req.ori.values, req.ori.mode );

		return true;
	}

	void requestChangeInDesiredPose( const geometry_msgs::Twist & msg )
	{
		seabee3_msgs::SetDesiredPose::Request req;
		seabee3_msgs::SetDesiredPose::Response resp;
		seabee3_msgs::Vector3Masked change_in_desired_pose;

		change_in_desired_pose.mask.x = change_in_desired_pose.mask.y = change_in_desired_pose.mask.z = 1; //enable all values
		change_in_desired_pose.mode.x = change_in_desired_pose.mode.y = change_in_desired_pose.mode.z = 1; //set mode to incremental

		req.pos = req.ori = change_in_desired_pose;

		req.pos.values = msg.linear;
		req.ori.values = msg.angular;

		setDesiredPoseCB( req, resp );
	}

	virtual void spinOnce()
	{
		if ( last_update_time_ != ros::Time( -1 ) )
		{
			ros::Duration dt = ros::Time::now() - last_update_time_;


			//twistCache is essentially the linear and angular change in desired pose per second (linear + angular velocity)
			//we multiply by the change in time and an arbitrary constant to obtain the desired change in pose
			//this change in pose is then added to the current pose; the result is desiredPose
			const double t1 = dt.toSec() * -100.0;
			geometry_msgs::Twist changeInDesiredPose = twist_cache_ * t1;
			requestChangeInDesiredPose( changeInDesiredPose );

			desired_pose_ >> desired_pose_tf_;
			tb_->sendTransform( tf::StampedTransform( desired_pose_tf_, ros::Time::now(), global_frame_, "seabee3/desired_pose" ) );

		}
		last_update_time_ = ros::Time::now();

		ros::Rate( 100 ).sleep();
	}
};

int main( int argc, char** argv )
{
	ros::init( argc, argv, "dead_reckoning_odom" );
	ros::NodeHandle nh;

	OdomNode odom_node( nh );
	odom_node.spin( BaseNode::SpinModeId::loop_spin_once );

	return 0;
}
