#include <base_libs/macros.h>
#include <base_libs/types.h>
#include <base_libs/image_proc_policy.h>

class TestImageProcPolicy : public BASE_LIBS_Node<base_libs::ImageProcPolicy>
{
public:
	TestImageProcPolicy( ros::NodeHandle & nh ) : BASE_LIBS_Node<base_libs::ImageProcPolicy>( nh )
	{
		
	}
	
	IMAGE_PROC_PROCESS_IMAGE( image_ptr )
	{
		IplImage * image = &IplImage( image_ptr->image );
		publishImage( image_ptr, "output_image" );
	}
	
	void spinOnce()
	{
		//
	}
};

BASE_LIBS_DECLARE_NODE( TestImageProcPolicy, "test_image_proc_policy" )
