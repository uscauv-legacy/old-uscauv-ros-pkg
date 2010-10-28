#include <base_image_proc/base_image_proc.h>

class Demo1: public BaseImageProc<>
{
public:
	Demo1( ros::NodeHandle & nh ) :
		BaseImageProc<> ( nh )
	{
		//
	}

	virtual cv::Mat processImage( IplImage * ipl_img )
	{
		ROS_INFO( "Processed image" );
		cv_img_ = cv::Mat( ipl_img );

		std::vector<cv::Mat> channels;
		cv::split(cv_img_, channels);
		
		channels.erase(channels.begin());
		channels.erase(channels.begin());

		cv::Mat blank_channel = cv::Mat::zeros(cv_img_.size(), CV_8U);
		channels.insert(channels.begin(), blank_channel);
		channels.insert(channels.begin(), blank_channel);

		cv::merge(channels, cv_img_);
		return cv_img_;
	}

};

int main( int argc, char ** argv )
{
	ros::init( argc, argv, "demo1_sagar" );
	ros::NodeHandle nh;

	Demo1 demo1( nh );
	demo1.spin();

	return 0;
}
