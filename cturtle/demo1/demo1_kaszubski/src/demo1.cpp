#include <base_image_proc/base_image_proc.h>

typedef BaseImageProcSettings::_DefaultReconfigureType _ConfigType;
class Demo1 : public BaseImageProc<_ConfigType>
{
public:
	Demo1( ros::NodeHandle & nh ) : BaseImageProc<_ConfigType>( nh )
	{
		//
	}
	
	virtual cv::Mat processImage( IplImage * ipl_img )
	{
		ROS_INFO("Processed Image");
		cv_img_ = cv::Mat( ipl_img );
		
		cv::line( cv_img_, cv::Point( 0, 0 ), cv::Point( 50, 50 ), cv::Scalar( 255, 0, 0 ) );
		
		return cv_img_;
	}
	
};

int main( int argc, char ** argv )
{
	ros::init( argc, argv, "demo1_kaszubski" );
	ros::NodeHandle nh;
	
	Demo1 demo1( nh );
	demo1.spin();
	
	return 0;
}
