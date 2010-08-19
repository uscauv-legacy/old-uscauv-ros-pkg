#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <boost/thread.hpp>
#include <sensor_msgs/CameraInfo.h>
#include <opencv/cv.h>
#include <cv_bridge/CvBridge.h>
#include <sensor_msgs/Image.h>
#include <string>

boost::mutex img_mutex_l, img_mutex_r, info_mutex_l, info_mutex_r, flag_mutex;

bool new_left_img = false;
bool new_right_img = false;
bool new_left_info = false;
bool new_right_info = false;

sensor_msgs::ImagePtr img_left, img_right;
sensor_msgs::CameraInfo info_left, info_right;
image_transport::Publisher *disparity_pub;
sensor_msgs::CvBridge bridge;

void process_images()
{
	if ( new_left_img && new_right_img && new_left_info && new_right_info )
	{
		ROS_INFO( "processing..." );
		boost::lock_guard<boost::mutex> limgguard( img_mutex_l );
		boost::lock_guard<boost::mutex> rimgguard( img_mutex_r );
		boost::lock_guard<boost::mutex> linfoguard( info_mutex_l );
		boost::lock_guard<boost::mutex> rinfoguard( info_mutex_r );

		cv::Mat leftImage( bridge.imgMsgToCv( img_left ) );
		ROS_INFO( "made left image" );

		cv::Mat rightImage( bridge.imgMsgToCv( img_right ) );
		ROS_INFO( "made right image" );

		cv::Mat combined( cv::Size( leftImage.size().width * 2, std::max( leftImage.size().height, rightImage.size().height ) ), leftImage.type(), cv::Scalar( 0 ) );
		ROS_INFO( "made combined image" );

		//leftImage.copyTo( combined );
		ROS_INFO( "copied left image to combined image" );

		disparity_pub->publish( bridge.cvToImgMsg( & ( (IplImage) combined ) ) );


		//Reset new flags
		new_left_img = false;
		new_right_img = false;
		new_left_info = false;
		new_right_info = false;

		ROS_INFO( "done" );
	}
}

void img_cb_l( const sensor_msgs::ImageConstPtr& msg )
{
	ROS_INFO( "got left img" );
	img_mutex_l.lock();
	img_left = boost::const_pointer_cast<sensor_msgs::Image>( msg );
	//img_left = msg;

	img_mutex_l.unlock();

	boost::lock_guard<boost::mutex> guard( flag_mutex );
	new_left_img = true;

	process_images();
}

void img_cb_r( const sensor_msgs::ImageConstPtr& msg )
{
	ROS_INFO( "got right img" );
	img_mutex_r.lock();
	img_right = boost::const_pointer_cast<sensor_msgs::Image>( msg );
	//img_right = msg;

	img_mutex_r.unlock();

	boost::lock_guard<boost::mutex> guard( flag_mutex );
	new_right_img = true;

	process_images();
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

	process_images();
}

void info_cb_r( const sensor_msgs::CameraInfoConstPtr& msg )
{
	ROS_INFO( "got right info" );
	info_mutex_r.lock();
	info_right = *msg;

	info_mutex_r.unlock();

	boost::lock_guard<boost::mutex> guard( flag_mutex );
	new_right_info = true;

	process_images();
}

int main( int argc, char * argv[] )
{
	ros::init( argc, argv, "stereo_proc" );
	ros::NodeHandle n;
	ros::NodeHandle n_priv( "~" );

	std::string stereo_ns, image, left_ns, right_ns;

	image_transport::Subscriber img_sub_l, img_sub_r;
	ros::Subscriber info_sub_l, info_sub_r;
	image_transport::ImageTransport it( n_priv );

	n_priv.param( "stereo", stereo_ns, std::string( "/stereo" ) );
	n_priv.param( "image", image, std::string( "image_raw" ) );
	n_priv.param( "left", left_ns, std::string( "left" ) );
	n_priv.param( "right", right_ns, std::string( "right" ) );

	img_sub_l = it.subscribe( stereo_ns + "/" + left_ns + "/" + image, 1, img_cb_l );
	img_sub_r = it.subscribe( stereo_ns + "/" + right_ns + "/" + image, 1, img_cb_r );

	info_sub_l = n_priv.subscribe( stereo_ns + "/" + left_ns + "/camera_info", 1, info_cb_l );
	info_sub_r = n_priv.subscribe( stereo_ns + "/" + right_ns + "/camera_info", 1, info_cb_r );

	ROS_INFO( ( stereo_ns + "/" + left_ns + "/camera_info" ).c_str() );

	disparity_pub = new image_transport::Publisher;
	*disparity_pub = it.advertise( stereo_ns + "/disparity", 1 );

	ros::spin();
}
