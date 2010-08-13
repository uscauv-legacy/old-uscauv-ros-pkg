#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <boost/thread.hpp>
#include <sensor_msgs/CameraInfo.h>

boost::mutex img_mutex_l, img_mutex_r, info_mutex_l, info_mutex_r, flag_mutex;

bool new_left_img = false;
bool new_right_img = false;
bool new_left_info = false;
bool new_right_info = false;

sensor_msgs::ImagePtr img_left, img_right;
sensor_msgs::CameraInfo info_left, info_right;
image_transport::Publisher *l_img_pub, *r_img_pub;
ros::Publisher *l_info_pub, *r_info_pub;

void publish_syncd_imgs()
{
	if ( new_left_img && new_right_img && new_left_info && new_right_info )
	{
		ROS_INFO( "publishing..." );
		boost::lock_guard<boost::mutex> limgguard( img_mutex_l );
		boost::lock_guard<boost::mutex> rimgguard( img_mutex_r );
		boost::lock_guard<boost::mutex> linfoguard( info_mutex_l );
		boost::lock_guard<boost::mutex> rinfoguard( info_mutex_r );

		ros::Time now = ros::Time::now();

//		ROS_INFO( "setting stamps" );

		img_left->header.stamp = now;
		img_right->header.stamp = now;
		info_left.header.stamp = now;
		info_right.header.stamp = now;

//		ROS_INFO( "publishing data" );

		l_img_pub->publish( img_left );
		r_img_pub->publish( img_right );
		l_info_pub->publish( info_left );
		r_info_pub->publish( info_right );

//		ROS_INFO( "resetting flags" );

		//Reset new flags
		new_left_img = false;
		new_right_img = false;
		new_left_info = false;
		new_right_info = false;

		ROS_INFO( "done" );
	}
//	ROS_INFO( "still waiting on all data to be syncd... %d %d %d %d", new_left_img, new_right_img, new_left_info, new_right_info );
}

void img_cb_l( const sensor_msgs::ImageConstPtr& msg )
{
	ROS_INFO( "got left img" );
	img_mutex_l.lock();
	img_left = boost::const_pointer_cast<sensor_msgs::Image>( msg );

	img_mutex_l.unlock();

	boost::lock_guard<boost::mutex> guard( flag_mutex );
	new_left_img = true;

	publish_syncd_imgs();
}

void img_cb_r( const sensor_msgs::ImageConstPtr& msg )
{
	ROS_INFO( "got right img" );
	img_mutex_r.lock();
	img_right = boost::const_pointer_cast<sensor_msgs::Image>( msg );

	img_mutex_r.unlock();

	boost::lock_guard<boost::mutex> guard( flag_mutex );
	new_right_img = true;

	publish_syncd_imgs();
}

void info_cb_l( const sensor_msgs::CameraInfoConstPtr& msg )
{
	ROS_INFO( "got left info" );
	info_mutex_l.lock();
	info_left = *msg;
	//info_left = boost::const_pointer_cast<sensor_msgs::CameraInfo>(msg);

	info_mutex_l.unlock();

	boost::lock_guard<boost::mutex> guard( flag_mutex );
	new_left_info = true;

	publish_syncd_imgs();
}

void info_cb_r( const sensor_msgs::CameraInfoConstPtr& msg )
{
	ROS_INFO( "got right info" );
	info_mutex_r.lock();
	info_right = *msg;


	//ROS_INFO( "%s", msg->header.stamp );
	//ROS_INFO( "%s", info_right.header.stamp );

	//info_right = boost::const_pointer_cast<sensor_msgs::CameraInfo>(msg);

	info_mutex_r.unlock();

	boost::lock_guard<boost::mutex> guard( flag_mutex );
	new_right_info = true;

	publish_syncd_imgs();
}

int main( int argc, char * argv[] )
{
	ros::init( argc, argv, "image_sync" );
	ros::NodeHandle n;
	ros::NodeHandle n_priv( "~" );

	image_transport::Subscriber img_sub_l, img_sub_r;
	ros::Subscriber info_sub_l, info_sub_r;
	image_transport::ImageTransport it( n_priv );

	img_sub_l = it.subscribe( "image_l", 1, img_cb_l );
	img_sub_r = it.subscribe( "image_r", 1, img_cb_r );

	info_sub_l = n_priv.subscribe( "info_l", 1, info_cb_l );
	info_sub_r = n_priv.subscribe( "info_r", 1, info_cb_r );

	l_img_pub = new image_transport::Publisher;
	*l_img_pub = it.advertise( "image_l_sync", 1 );

	r_img_pub = new image_transport::Publisher;
	*r_img_pub = it.advertise( "image_r_sync", 1 );

	l_info_pub = new ros::Publisher;
	*l_info_pub = n_priv.advertise<sensor_msgs::CameraInfo> ( "info_l_sync", 1 );

	r_info_pub = new ros::Publisher;
	*r_info_pub = n_priv.advertise<sensor_msgs::CameraInfo> ( "info_r_sync", 1 );


	//	img_left = new sensor_msgs::Image;
	//	img_right = new sensor_msgs::Image;

	ros::spin();
}
