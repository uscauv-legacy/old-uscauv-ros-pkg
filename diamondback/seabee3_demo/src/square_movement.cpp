#include <base_node/base_node.h>
#include <geometry_msgs/Twist.h>
#include <mathy_math/mathy_math.h>

// srvs
#include <seabee3_common/SetDesiredPose.h> // for SetDesiredPose
class SquareMovement : public BaseNode<>
{

protected:
	geometry_msgs::Twist cmd_vel_;
	seabee3_common::SetDesiredPose set_desired_pose_;
	ros::Publisher cmd_vel_pub_;
	ros::ServiceClient set_desired_heading_cli_;

public:
	SquareMovement( ros::NodeHandle & nh ) :
		BaseNode<> ( nh )
	{
		cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist> ( "/seabee3/cmd_vel", 1 );
		set_desired_heading_cli_ = nh_priv_.serviceClient<seabee3_common::SetDesiredPose> ( "/seabee3/set_desired_pose" );
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

	void setYaw( float yaw, float duration )
	{
		if( !ros::ok() ) return;

		printf( "Setting yaw to %f for %f\n", yaw, duration );
		ros::Time end_time = ros::Time::now() + ros::Duration( duration );

		// mode defaults to absolute
		// enable the mask for yaw
		set_desired_pose_.request.ori.mask.z = 1;
		// set the yaw
		set_desired_pose_.request.ori.values.z = MathyMath::degToRad( yaw );
		// publish
		set_desired_heading_cli_.call( set_desired_pose_.request, set_desired_pose_.response );

		while ( ros::ok() && ros::Time::now() < end_time )
		{
			ros::Rate( 10 ).sleep();
			ros::spinOnce();
		}
	}

	void spin()
	{
		static float forward_velocity = 0.5;
		static float reverse_velocity = -0.3;
		static float reverse_duration = 1.0;
		while ( ros::ok() )
		{
			setYaw( 0, 4 );
			setVelocity( forward_velocity, 8 );

			setVelocity( reverse_velocity, reverse_duration );
			setVelocity( 0, 2 );
			setYaw( 90, 4 );
			setVelocity( forward_velocity, 8 );

			setVelocity( reverse_velocity, reverse_duration );
			setVelocity( 0, 2 );
			setYaw( 180, 4 );
			setVelocity( forward_velocity, 8 );

			setVelocity( reverse_velocity, reverse_duration );
			setVelocity( 0, 2 );
			setYaw( 270, 4 );
			setVelocity( forward_velocity, 8 );

			setVelocity( reverse_velocity, reverse_duration );
			setVelocity( 0, 2 );
		}
	}

};

int main( int argc, char ** argv )
{
	ros::init( argc, argv, "square_movement" );
	ros::NodeHandle nh;

	SquareMovement square_movement( nh );
	square_movement.spin();

	return 0;
}
