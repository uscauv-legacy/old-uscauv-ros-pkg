#ifndef BASE_LIBS_BASE_LIBS_JOYSTICK_POLICY_H_
#define BASE_LIBS_BASE_LIBS_JOYSTICK_POLICY_H_

#include <base_libs/policy.h>

namespace base_libs
{

BASE_LIBS_DECLARE_POLICY( Joystick, Policy )

BASE_LIBS_DECLARE_POLICY_CLASS( Joystick )
{
	BASE_LIBS_MAKE_POLICY_NAME( Joystick )
	
	BASE_LIBS_DECLARE_POLICY_CONSTRUCTOR( Joystick )
	{
		printPolicyActionStart( "create", this );
		printPolicyActionDone( "create", this );
	}

};

}

#endif // BASE_LIBS_BASE_LIBS_JOYSTICK_POLICY_H_
