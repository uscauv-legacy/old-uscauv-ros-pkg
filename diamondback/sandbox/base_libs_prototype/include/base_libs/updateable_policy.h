#ifndef BASE_LIBS_BASE_LIBS_UPDATEABLE_POLICY_H_
#define BASE_LIBS_BASE_LIBS_UPDATEABLE_POLICY_H_

#include <ros/node_handle.h>
#include <ros/rate.h>
#include <base_libs/param_reader.h>
#include <base_libs/policy.h>
#include <base_libs/macros.h>

namespace base_libs
{

class UpdateablePolicy : public Policy
{	
public:
	template<class... __Args>
	UpdateablePolicy( __Args&&... args )
	:
		Policy( args... )
	{
		ROS_INFO( "Creating updateable policy..." );
		ROS_INFO( "Done creating updateable policy." );
	}
	
	BASE_LIBS_ENABLE_UPDATE{}
};

}

#endif // BASE_LIBS_BASE_LIBS_UPDATEABLE_POLICY_H_
