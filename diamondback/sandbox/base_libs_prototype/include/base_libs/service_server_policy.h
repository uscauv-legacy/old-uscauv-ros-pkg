#ifndef BASE_LIBS_BASE_LIBS_SERVICE_SERVER_POLICY_H_
#define BASE_LIBS_BASE_LIBS_SERVICE_SERVER_POLICY_H_

#include <base_libs/node_handle_policy.h>
#include <ros/service_server.h>

namespace base_libs
{

BASE_LIBS_DECLARE_POLICY( ServiceServer, NodeHandlePolicy )

BASE_LIBS_DECLARE_POLICY_CLASS( ServiceServer )
{
	BASE_LIBS_MAKE_POLICY_NAME( ServiceServer )
	
	BASE_LIBS_DECLARE_POLICY_CONSTRUCTOR( ServiceServer )
	{
		printPolicyActionStart( "create", this );
		printPolicyActionDone( "create", this );
	}
	
};

}

#endif // BASE_LIBS_BASE_LIBS_SERVICE_SERVER_POLICY_H_
