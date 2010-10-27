#include <base_image_proc/base_image_proc.h>

#include <iostream>
#include <iomanip>
#include <sstream>
#include <vector>

class ImageServer: public BaseImageProc<>
{
private:
	std::vector<cv::Mat> image_cache_;

	std::string file_prefix_;
	std::string file_ext_;
	int start_;
	int end_;
	int digits_;
	double rate_;
	bool loop_;

public:
	ImageServer( ros::NodeHandle & nh ) :
		BaseImageProc<> ( nh )
	{
		nh_priv_.param( "prefix", file_prefix_, std::string( "" ) );
		nh_priv_.param( "start", start_, 0 );
		nh_priv_.param( "end", end_, 0 );
		nh_priv_.param( "digits", digits_, 0 );
		nh_priv_.param( "ext", file_ext_, std::string( "" ) );
		nh_priv_.param( "rate", rate_, 15.0 );
		nh_priv_.param( "loop", loop_, false );
	}

	~ImageServer()
	{
		delete img_pub_;
	}

	virtual void spinOnce()
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

				if ( img.data != NULL )
				{
					image_cache_.push_back( img );
				}
				else
				{
					ROS_WARN( "Ignoring %s; does not exist in filesystem", filename.str().c_str() );
					end_--;
					current_frame--;
					continue;
				}
			}

			//Memory leak? Probably.
			IplImage ipl_img = img;
			publishCvImage( &ipl_img );

			ros::Rate( rate_ ).sleep();
			current_frame++;

			if ( current_frame > end_ && loop_ ) current_frame = start_;
		}
	}
};

void printUsage()
{
	printf( "\nUsage: image_server [ prefix start end digits ext rate loop ]\n" );
}

int main( int argc, char* argv[] )
{
	printf( "argc: %d", argc );
	if ( argc < 2 )
	{
		printUsage();
		return 0;
	}

	for ( int i = 1; i < argc; i++ )
	{
		if ( strcmp( argv[i], "-h" ) == 0 || strcmp( argv[i], "--help" ) == 0 )
		{
			printUsage();
			return 0;
		}
	}

	ros::init( argc, argv, "image_server" );
	ros::NodeHandle nh;

	ImageServer image_server( nh );
	image_server.spin( SpinModeId::loop_spin_once );

	return 0;
}

