#include <ros/ros.h>
// for ImageTransport, Publisher
#include <image_transport/image_transport.h>
// for Mat
#include <opencv/cv.h>
// for imread()
#include "highgui.h"
//for CvBridge
#include <cv_bridge/CvBridge.h>

#include <iostream>
#include <iomanip>
#include <sstream>
#include <vector>

class ImageServer
{
private:
	ros::NodeHandle nh_priv_;
	sensor_msgs::CvBridge cv_bridge_;
	image_transport::ImageTransport it_;
	image_transport::Publisher * img_pub_;
	std::vector<cv::Mat> image_cache_;

	std::string file_prefix_;
	std::string file_ext_;
	int start_;
	int end_;
	int digits_;
	int rate_;
	bool loop_;

public:
	ImageServer( ros::NodeHandle & nh ) :
		nh_priv_( "~" ), it_( nh_priv_ )
	{
		nh_priv_.param( "prefix", file_prefix_, std::string( "" ) );
		nh_priv_.param( "start", start_, 0 );
		nh_priv_.param( "end", end_, 0 );
		nh_priv_.param( "digits", digits_, 0 );
		nh_priv_.param( "ext", file_ext_, std::string( "" ) );
		nh_priv_.param( "rate", rate_, 15 );
		nh_priv_.param( "loop", loop_, false );

		img_pub_ = new image_transport::Publisher( it_.advertise( "image_color", 1 ) );
	}

	~ImageServer()
	{
		delete img_pub_;
	}

	void spin()
	{
		int current_frame = start_;
		while ( ros::ok() && current_frame <= end_ )
		{
			cv::Mat img;
			if ( image_cache_.size() > 0 && image_cache_.size() > current_frame - start_ )
			{
				img = image_cache_[current_frame - start_];
			}
			else
			{
				std::stringstream filename;
				filename << file_prefix_ << std::setfill( '0' ) << std::setw( digits_ ) << current_frame << file_ext_;
				ROS_INFO( "Opening %s", filename.str().c_str() );
				img = cv::imread( filename.str().c_str() );
				image_cache_.push_back( img );
			}

			//Memory leak? Probably.
			IplImage ipl_img = img;
			img_pub_->publish( cv_bridge_.cvToImgMsg( &ipl_img ) );

			ros::spinOnce();
			ros::Rate( rate_ ).sleep();
			current_frame++;

			if ( current_frame > end_ && loop_ ) current_frame = start_;
		}
	}
};

int main( int argc, char* argv[] )
{
	ros::init( argc, argv, "image_server" );
	ros::NodeHandle nh;

	ImageServer image_server( nh );
	image_server.spin();

	return 0;
}

