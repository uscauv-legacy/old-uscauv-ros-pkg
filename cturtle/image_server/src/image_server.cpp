#include <base_image_proc/base_image_proc.h>
#include <image_loader/image_loader.h>

class ImageServer : public BaseImageProc<>
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

	virtual ~ImageServer()
	{
	}

	void spinOnce()
	{
		static int current_frame = 0;

		if ( image_loader_.images_loaded_ && current_frame < image_loader_.image_cache_.size() )
		{
			ROS_INFO( "Publishing image %d", current_frame );
			publishCvImage( image_loader_.image_cache_[current_frame] );
			++current_frame;
		}
		else if( loop_ )
		{
			current_frame = 0;
		}

		ros::Rate( rate_ ).sleep();
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
	image_server.spin( SpinModeId::LOOP_SPIN_ONCE );

	return 0;
}

