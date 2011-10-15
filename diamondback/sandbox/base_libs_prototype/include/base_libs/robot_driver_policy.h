#ifndef BASE_LIBS_BASE_LIBS_ROBOT_DRIVER_POLICY_H_
#define BASE_LIBS_BASE_LIBS_ROBOT_DRIVER_POLICY_H_

#include <base_libs/policy.h>

namespace base_libs
{

BASE_LIBS_DECLARE_POLICY( RobotDriver, Policy )

BASE_LIBS_DECLARE_POLICY_CLASS( RobotDriver )
{
	BASE_LIBS_MAKE_POLICY_NAME( RobotDriver )
	
	BASE_LIBS_DECLARE_POLICY_CONSTRUCTOR( RobotDriver )
	{
		printPolicyActionStart( "create", this );
		printPolicyActionDone( "create", this );
	}

};

}

#endif // BASE_LIBS_BASE_LIBS_ROBOT_DRIVER_POLICY_H_
