#include <base_image_proc/base_image_proc.h>
#include <image_loader/image_loader.h>

class ImageServer: public BaseImageProc<>
{
private:
	ImageLoader image_loader_;

	double rate_;
	bool loop_;

public:
	ImageServer( ros::NodeHandle & nh ) :
		BaseImageProc<> ( nh ), image_loader_( nh_priv_ )
	{
		nh_priv_.param( "rate", rate_, 15.0 );
		nh_priv_.param( "loop", loop_, false );

		image_loader_.loadImages();
	}

	~ImageServer()
	{
		delete img_pub_;
	}

	void spinOnce()
	{
		static int current_frame = image_loader_.start_;

		ROS_INFO("spinning...");

		if ( image_loader_.images_loaded_ && image_loader_.image_cache_.size() > 0 && current_frame <= image_loader_.end_ )
		{
			ROS_INFO( "Publishing image %d index %d", current_frame, current_frame - image_loader_.start_ );
			publishCvImage( &(image_loader_.image_cache_[current_frame - image_loader_.start_]) );
		}

		ros::Rate( rate_ ).sleep();
		current_frame++;

		if ( current_frame > image_loader_.end_ && loop_ ) current_frame = image_loader_.start_;
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

