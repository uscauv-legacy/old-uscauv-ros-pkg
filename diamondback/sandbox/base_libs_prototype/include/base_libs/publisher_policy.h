#ifndef BASE_LIBS_BASE_LIBS_PUBLISHER_POLICY_H_
#define BASE_LIBS_BASE_LIBS_PUBLISHER_POLICY_H_

#include <base_libs/node_handle_policy.h>
#include <base_libs/multi_publisher.h>

namespace base_libs
{

BASE_LIBS_DECLARE_POLICY( Publisher, NodeHandlePolicy )

template<class __Publisher = ros::Publisher>
BASE_LIBS_DECLARE_POLICY_CLASS( Publisher )
{
	BASE_LIBS_MAKE_POLICY_NAME( Publisher )
	
public:
	ros::MultiPublisher<__Publisher> publishers_;
	
	BASE_LIBS_DECLARE_POLICY_CONSTRUCTOR( Publisher )
	{
		printPolicyActionStart( "create", this );
		printPolicyActionDone( "create", this );
	}
};

}

#endif // BASE_LIBS_BASE_LIBS_PUBLISHER_POLICY_H_
