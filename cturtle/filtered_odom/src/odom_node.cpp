#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>

geometry_msgs::Twist pose;
tf::Transform odom_frame;
tf::TransformBroadcaster * odom_broadcaster;

bool resetPoseCB( seabee3_dead_reckoning::ResetOdom::Request & req, seabee3_dead_reckoning::ResetOdom::Response & resp)
{
	x = req.Pos.x;
	y = req.Pos.y;
	z = req.Pos.z; //this is just depth
	odom_trans = tf::Transform( tf::Quaternion( ori.z, ori.y, ori.x), tf::Vector3( x, y, z) );
	return true;
}

void odom_inc_dro_callback( const geometry_msgs::TwistConstPtr & msg )
{
	updatePose( * msg );
}

void odom_inc_sio_callback( const geometry_msgs::TwistConstPtr & msg )
{
	updatePose( * msg );
}

//TODO: make sure to properly combine stereo and dead-reckoning odom
void updatePose( geometry_msgs::Twist & msg )
{
	pose.linear += msg.linear;
	pose.angular += msg.angular;
	odom_frame = tf::Transform( tf::Quaternion( pose.angular.z, pose.angular.y, pose.angular.x), tf::Vector3( pose.linear.x, pose.linear.y, pose.linear.z) );
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "filtered_odom");
	ros::NodeHandle n;
	ros::NodeHandle n_priv("~");

	ros::ServiceServer ResetOdomCallback = n.advertiseService ("/seabee3/ResetOdom", ResetOdomCallback);

	odom_broadcaster = new tf::TransformBroadcaster;

	while( ros::ok() )
	{
		odom_broadcaster->sendTransform( tf::StampedTransform( odom_frame, ros::Time::now(), "/seabee3/landmark_map", "/seabee3/odom" ) );
		ros::spinOnce();
		ros::Rate(20).sleep();
	}

	return 0;
}
