#ifndef BASE_LIBS_BASE_LIBS_SUBSCRIBER_POLICY_H_
#define BASE_LIBS_BASE_LIBS_SUBSCRIBER_POLICY_H_

#include <base_libs/node_handle_policy.h>
#include <base_libs/generic_policy_adapter.h>
#include <base_libs/multi_subscriber.h>
#include <base_libs/macros.h>

namespace base_libs
{

BASE_LIBS_DECLARE_POLICY( Subscriber, NodeHandlePolicy )

BASE_LIBS_DECLARE_POLICY_CLASS( Subscriber )
{
	BASE_LIBS_MAKE_POLICY_NAME( Subscriber )
	
protected:
	ros::MultiSubscriber<> multi_sub_;
	
	BASE_LIBS_DECLARE_POLICY_CONSTRUCTOR( Subscriber )
	{
		printPolicyActionStart( "create", this );
		printPolicyActionDone( "create", this );
	}
};

}

#endif // BASE_LIBS_BASE_LIBS_SUBSCRIBER_POLICY_H_
