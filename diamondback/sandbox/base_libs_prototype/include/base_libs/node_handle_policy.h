#ifndef BASE_LIBS_BASE_LIBS_NODE_HANDLE_POLICY_H_
#define BASE_LIBS_BASE_LIBS_NODE_HANDLE_POLICY_H_

#include <ros/node_handle.h>
#include <base_libs/policy.h>
#include <base_libs/generic_policy_adapter.h>
#include <base_libs/type_utils.h>
#include <base_libs/macros.h>

namespace base_libs
{

BASE_LIBS_DECLARE_POLICY( NodeHandle, Policy )

BASE_LIBS_DECLARE_POLICY_CLASS( NodeHandle )
{
	BASE_LIBS_MAKE_POLICY_NAME( NodeHandle )
	
protected:
	ros::NodeHandle nh_rel_;
	
	BASE_LIBS_DECLARE_POLICY_CONSTRUCTOR( NodeHandle ),
		nh_rel_( getFirstOfType<ros::NodeHandle>( args... ) )
	{
		printPolicyActionStart( "create", this );
		printPolicyActionDone( "create", this );
	}
};

}

#endif // BASE_LIBS_BASE_LIBS_NODE_HANDLE_POLICY_H_
