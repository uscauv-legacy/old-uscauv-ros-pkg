#include <base_libs/macros.h>
#include <base_libs/node.h>
#include <base_libs/image_proc_policy.h>

BASE_LIBS_DECLARE_NODE( TestImageProcPolicy, base_libs::ImageProcPolicy )
//typedef base_libs::Node<base_libs::ImageProcPolicy> _Node;
BASE_LIBS_DECLARE_NODE_CLASS( TestImageProcPolicy )
//class TestImageProcPolicy : public _Node
{
	BASE_LIBS_DECLARE_NODE_CONSTRUCTOR( TestImageProcPolicy )
//public:
//	template<class... __Args>
//	TestImageProcPolicy( __Args&&... args ) : _Node( args... )
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
