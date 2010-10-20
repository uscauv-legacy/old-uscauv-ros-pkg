#include <base_image_proc/base_image_proc.h>

class Demo1: public BaseImageProc<>
{
public:
	Demo1( ros::NodeHandle & nh ) :
		BaseImageProc<> ( nh )
	{

	}
	
	virtual cv::Mat processImage( IplImage * ipl_img )
	{
		ROS_INFO( "Prroccesed Image" );
		cv_img_ = cv::Mat( ipl_img );

		cv::circle( cv_img_, cv::Point( 150, 270 ), 100, cv::Scalar( 0, 255, 67 ) );

		return cv_img_;
	}
};

int main( int argc, char ** argv )
{
	ros::init( argc, argv, "demo1_sung" );
	ros::NodeHandle nh;

	Demo1 demo1( nh );
	demo1.spin();

	return 0;

}

