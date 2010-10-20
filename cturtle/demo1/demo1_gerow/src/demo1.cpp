#include <base_image_proc/base_image_proc.h>
#include <time.h>
class Demo1: public BaseImageProc<>
{
public:
	Demo1( ros::NodeHandle & nh ) :
		BaseImageProc<> ( nh )
	{
		reconfigure_initialized_ = true;
	}
	
	virtual cv::Mat processImage( IplImage * ipl_img )
	{
		cv_img_ = cv::Mat( ipl_img );
		
		cv::Size size;
		cv::Point center;
		long radius, time;
		cv::Scalar color;
		
		time = ( clock() * 10 ) / CLOCKS_PER_SEC;
		
		size = cv_img_.size();
		
		center.x = size.width / 2;
		center.y = size.height / 2;
		if ( size.width > size.height )
		{
			radius = size.height / 4;
		}
		else
		{
			radius = size.width / 4;
		}

		radius = radius / ( ( time % 10 ) + 1 );
		
		if ( ( time % 3 ) == 0 )
		{
			color = cv::Scalar( 0, 255, 0 );
		}
		else if ( ( time % 3 ) == 2 )
		{
			color = cv::Scalar( 0, 0, 255 );
		}
		else
		{
			color = cv::Scalar( 255, 0, 0 );
		}

		cv::circle( cv_img_, center, radius, color, 4 );
		cv::rectangle( cv_img_, cv::Point( 5, 5 ), cv::Point( 20, 20 ), cv::Scalar( 0, 255, 0 ) );
		cv::putText( cv_img_, ":)", center, 2, 1, cv::Scalar( 255, 0, 0 ) );
		//cv::line(cv_img_, cv::Point( 0, 0 ), cv::Point(50, 50), cv::Scalar( 255, 0, 0));

		return cv_img_;
	}

};

int main( int argc, char **argv )
{
	ros::init( argc, argv, "demo1_gerow" );
	ros::NodeHandle nh;
	
	Demo1 demo1( nh );
	demo1.spin();
	
	return 0;
}
