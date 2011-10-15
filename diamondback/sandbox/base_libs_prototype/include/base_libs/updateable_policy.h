#ifndef BASE_LIBS_BASE_LIBS_UPDATEABLE_POLICY_H_
#define BASE_LIBS_BASE_LIBS_UPDATEABLE_POLICY_H_

#include <base_libs/policy.h>

namespace base_libs
{

BASE_LIBS_DECLARE_POLICY( Updateable, Policy )

BASE_LIBS_DECLARE_POLICY_CLASS( Updateable )
{
	BASE_LIBS_MAKE_POLICY_NAME( Updateable )

	BASE_LIBS_DECLARE_POLICY_CONSTRUCTOR( Updateable )
	{
		printPolicyActionStart( "create", this );
		printPolicyActionDone( "create", this );
	}
	
	BASE_LIBS_ENABLE_UPDATE{}
};

}

#endif // BASE_LIBS_BASE_LIBS_UPDATEABLE_POLICY_H_
