#include <base_libs/macros.h>
#include <base_libs/node.h>
#include <base_libs/image_proc_policy.h>

BASE_LIBS_DECLARE_NODE( TestImageProcPolicy, base_libs::ImageProcPolicy )

BASE_LIBS_DECLARE_NODE_CLASS( TestImageProcPolicy )
{
	BASE_LIBS_DECLARE_NODE_CONSTRUCTOR( TestImageProcPolicy )
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

BASE_LIBS_INST_NODE( TestImageProcPolicyNode, "test_image_proc_policy_node" )
