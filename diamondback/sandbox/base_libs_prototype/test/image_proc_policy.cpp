#include <base_libs/macros.h>
#include <base_libs/node.h>
#include <base_libs/image_proc_policy.h>

class TestImageProcPolicy :
	public base_libs::Node<
		base_libs::ImageProcPolicy>
{
public:
	template<class... __Args>
	TestImageProcPolicy( __Args&&... args ) : base_libs::Node<base_libs::ImageProcPolicy>( args... )
	{
		
	}
	
	IMAGE_PROC_PROCESS_IMAGE( image_ptr )
	{
		IplImage * image = &IplImage( image_ptr->image );
		
		//
		
		publishImage( image_ptr, "output_image" );
	}
	
	void spinOnce()
	{
		//
	}
};

BASE_LIBS_DECLARE_NODE( TestImageProcPolicy, "test_image_proc_policy" )
