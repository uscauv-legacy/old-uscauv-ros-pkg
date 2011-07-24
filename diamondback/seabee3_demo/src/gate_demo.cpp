#include <base_node/base_node.h>
#include <geometry_msgs/Twist.h>
#include <common_utils/math.h>
#include <common_utils/tf.h>

// msgs
#include <seabee3_driver_base/KillSwitch.h>

// srvs
#include <seabee3_common/SetDesiredPose.h> // for SetDesiredPose
#include <std_srvs/Empty.h> // for ResetPose

class GateDemo : public BaseNode<>
{

protected:
	geometry_msgs::Twist cmd_vel_;
	ros::Publisher cmd_vel_pub_;
	ros::Subscriber kill_switch_sub_;
	ros::ServiceClient set_desired_heading_cli_;
	ros::ServiceClient reset_pose_cli_;

	double forward_velocity_;
	double forward_time_;
	double forward_velocity2_;
	double forward_time2_;
	double depth_;
	double dive_time_;
  double buoy_rotate_degrees_;
	tf::Transform current_pose_;

	bool kill_timers_;
	bool kill_behaviors_;
	bool running_;
  double initial_heading_;

public:
	GateDemo( ros::NodeHandle & nh ) :
		BaseNode<> ( nh ), kill_timers_( false ), kill_behaviors_( false ), running_( false )
	{
		nh_local_.param( "forward_velocity", forward_velocity_, 0.3 );
		nh_local_.param( "forward_time", forward_time_, 30.0 );
		nh_local_.param( "forward_velocity2", forward_velocity2_, 0.3 );
		nh_local_.param( "forward_time2", forward_time2_, 30.0 );
		nh_local_.param( "depth", depth_, 1.7 );
		nh_local_.param( "dive_time", dive_time_, 8.0 );
		nh_local_.param( "buoy_rotate_degrees", buoy_rotate_degrees_, 15.0 );

		cmd_vel_pub_ = nh_local_.advertise<geometry_msgs::Twist> ( "/seabee3/cmd_vel", 2 );
		kill_switch_sub_ = nh_local_.subscribe( "/seabee3/kill_switch", 2, &GateDemo::killSwitchCB, this );
		set_desired_heading_cli_ = nh_local_.serviceClient<seabee3_common::SetDesiredPose> ( "/seabee3/set_desired_pose" );
		reset_pose_cli_ = nh_local_.serviceClient<std_srvs::Empty> ( "/seabee3/reset_pose" );
	}

	void setVelocity( float velocity, float duration )
	{
		if( !ros::ok() ) return;

		printf( "Setting velocity to %f for %f\n", velocity, duration );
		ros::Time end_time = ros::Time::now() + ros::Duration( duration );

		// set the desired forward velocity
		cmd_vel_.linear.x = velocity;

		while ( !kill_timers_ && ros::ok() && ros::Time::now() < end_time )
		{
			// publish
			cmd_vel_pub_.publish( cmd_vel_ );
			ros::Rate( 20 ).sleep();
			ros::spinOnce();
		}
	}

	void reset()
	{
		kill_behaviors_ = true;
		kill_timers_ = true;
		ros::Duration( 0.1 ).sleep();
		std_srvs::Empty reset_request;
		reset_pose_cli_.call( reset_request );
		kill_timers_ = false;
	}

	void killSwitchCB( const seabee3_driver_base::KillSwitch::ConstPtr & kill_switch_msg )
	{
		static bool last_killed_state_ = kill_switch_msg->is_killed;
		bool current_killed_state = kill_switch_msg->is_killed;
		ROS_INFO( "Updated kill switch state %d -> %d", last_killed_state_, current_killed_state );
		if( !last_killed_state_ && current_killed_state )
		{
			ROS_INFO( "Resetting." );
			setVelocity( 0.0, 0.1 );
			reset();
			running_ = false;
		}
		if( last_killed_state_ && !current_killed_state )
		{
			std_srvs::Empty reset_request;
			reset_pose_cli_.call( reset_request );
			kill_behaviors_ = false;
			ROS_INFO( "Starting up." );
			// mode defaults to absolute
			// enable the mask for yaw
			seabee3_common::SetDesiredPose set_desired_pose_;
			set_desired_pose_.request.ori.mask.z = 1;
			set_desired_pose_.request.pos.mask.z = 1;
			// set the yaw
			double roll, pitch, initial_heading_;
			current_pose_.getBasis().getEulerYPR( initial_heading_, pitch, roll );
			set_desired_pose_.request.ori.values.z = initial_heading_;
			set_desired_pose_.request.pos.values.z = -depth_;
			// publish
			ROS_INFO( "Publishing desired pose; setting depth to %f and yaw to %f", set_desired_pose_.request.pos.values.z, initial_heading_  );
			set_desired_heading_cli_.call( set_desired_pose_.request, set_desired_pose_.response );

/*			tf::Transform desired_pose( current_pose_.getRotation(), tf::Vector3( 0, 0, -depth_ ) );
			tf_utils::publishTfFrame( desired_pose, "/landmark_map", "/seabee3/desired_pose" );*/
			running_ = true;
		}

		last_killed_state_ = current_killed_state;
	}

	void sleep( float duration )
	{
		ROS_INFO( "Sleeping for %f seconds", duration );
		ros::Time end_time = ros::Time::now() + ros::Duration( duration );
		while ( !kill_timers_ && ros::ok() && ros::Time::now() < end_time )
		{
			ros::Rate( 20 ).sleep();
			ros::spinOnce();
		}
	}

	void spinOnce()
	{
		ROS_INFO( "Updated current pose" );
		tf_utils::fetchTfFrame( current_pose_, "/landmark_map", "/seabee3/base_link" );

		if( running_ )
		{
			// wait while we dive
			if( kill_behaviors_ )
			{
				kill_behaviors_ = false;
				return;
			}
			ROS_INFO( "Waiting for seabee to dive..." );
			sleep( dive_time_ );

			// drive forward for 30 seconds
			if( kill_behaviors_ )
			{
				kill_behaviors_ = false;
				return;
			}
			ROS_INFO( "Setting foward velocity..." );
			setVelocity( forward_velocity_, forward_time_ );

			// stop moving foward
			if( kill_behaviors_ )
			{
				kill_behaviors_ = false;
				return;
			}
			setVelocity( 0.0, 0.5 );

			// surface
			if( kill_behaviors_ )
			{
				kill_behaviors_ = false;
				return;
			}
			ROS_INFO( "Rotating to face buoys..." );
			// mode defaults to absolute
			// enable the mask for yaw
			seabee3_common::SetDesiredPose set_desired_pose_;
			set_desired_pose_.request.ori.mask.z = 1;
			set_desired_pose_.request.ori.values.z = initial_heading_ + math_utils::degToRad( buoy_rotate_degrees_ );
			// publish
			set_desired_heading_cli_.call( set_desired_pose_.request, set_desired_pose_.response );

			// drive forward for 30 seconds
			if( kill_behaviors_ )
			{
				kill_behaviors_ = false;
				return;
			}
			ROS_INFO( "Setting foward velocity..." );
			setVelocity( forward_velocity2_, forward_time2_ );

/*			tf::Transform desired_pose( tf_utils::ZERO_QUAT, tf::Vector3( 0, 0, 0 ) );
			tf_utils::publishTfFrame( desired_pose, "/landmark_map", "/seabee3/desired_pose" );*/

			running_ = false;
			return;
		}
	}

};

int main( int argc, char ** argv )
{
	ros::init( argc, argv, "gate_demo" );
	ros::NodeHandle nh( "~" );

	GateDemo gate_demo( nh );
	gate_demo.spin();

	return 0;
}
