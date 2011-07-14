#include <base_node/base_node.h>
#include <geometry_msgs/Twist.h>
#include <common_utils/math.h>
#include <common_utils/tf.h>

// msgs
#include <seabee3_driver_base/KillSwitch.h>

class GateDemo : public BaseNode<>
{

protected:
	geometry_msgs::Twist cmd_vel_;
	ros::Publisher cmd_vel_pub_;
	ros::Subscriber kill_switch_sub_;
	ros::ServiceClient set_desired_heading_cli_;

	double forward_velocity_;
	double depth_;
	double forward_time_;
	double dive_time_;

	bool started_;
	bool running_;

public:
	GateDemo( ros::NodeHandle & nh ) :
		BaseNode<> ( nh ), started_( false ), running_( false )
	{
		nh_local_.param( "forward_velocity", forward_velocity_, 0.3 );
		nh_local_.param( "forward_time", forward_time_, 30.0 );
		nh_local_.param( "depth", depth_, 1.7 );
		nh_local_.param( "dive_time", dive_time_, 8.0 );

		cmd_vel_pub_ = nh_local_.advertise<geometry_msgs::Twist> ( "/seabee3/cmd_vel", 1 );
		kill_switch_sub_ = nh_local_.subscribe( "seabee3/kill_switch", 1, &GateDemo::killSwitchCB, this );
	}

	void setVelocity( float velocity, float duration )
	{
		if( !ros::ok() ) return;

		printf( "Setting velocity to %f for %f\n", velocity, duration );
		ros::Time end_time = ros::Time::now() + ros::Duration( duration );

		// set the desired forward velocity
		cmd_vel_.linear.x = velocity;

		while ( ros::ok() && ros::Time::now() < end_time )
		{
			// publish
			cmd_vel_pub_.publish( cmd_vel_ );
			ros::Rate( 10 ).sleep();
			ros::spinOnce();
		}
	}

	void killSwitchCB( const seabee3_driver_base::KillSwitch::ConstPtr & kill_switch_msg )
	{
		if( !kill_switch_msg->is_killed && !started_ )
		{
			started_ = true;
			tf::Transform current_pose;
			tf_utils::fetchTfFrame( current_pose, "/landark_map", "/seabee3/current_pose" );
			
			tf::Transform desired_pose( current_pose.getRotation(), tf::Vector3( 0, 0, -depth_ ) );
			tf_utils::publishTfFrame( desired_pose, "/landmark_map", "/seabee3/desired_pose" );
			running_ = true;
		}
	}

	void sleep( float duration )
	{
		ros::Time end_time = ros::Time::now() + ros::Duration( duration );
		while ( ros::ok() && ros::Time::now() < end_time )
		{
			ros::Rate( 10 ).sleep();
			ros::spinOnce();
		}
	}

	void spin()
	{
		while ( true )
		{
			if( running_ )
			{
				// wait while we dive
				sleep( dive_time_ );
	
				// drive forward for 30 seconds
				setVelocity( forward_velocity_, forward_time_ );

				// surface
				tf::Transform desired_pose( tf_utils::ZERO_QUAT, tf::Vector3( 0, 0, 0 ) );
				tf_utils::publishTfFrame( desired_pose, "/landmark_map", "/seabee3/desired_pose" );
				return;
			}
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
