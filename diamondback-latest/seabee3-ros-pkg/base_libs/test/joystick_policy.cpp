#include <base_libs/node.h>
#include <base_libs/joystick_policy.h>

BASE_LIBS_DECLARE_NODE( TestJoystickPolicy, base_libs::JoystickPolicy )

BASE_LIBS_DECLARE_NODE_CLASS( TestJoystickPolicy )
{
	BASE_LIBS_DECLARE_NODE_CONSTRUCTOR( TestJoystickPolicy ){}
};

BASE_LIBS_INST_NODE( TestJoystickPolicyNode, "test_joystick_policy_node" )
