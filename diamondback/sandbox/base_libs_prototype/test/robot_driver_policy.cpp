#include <base_libs/robot_driver_policy.h>
#include <base_libs/node.h>

BASE_LIBS_DECLARE_NODE( TestRobotDriverPolicy, base_libs::RobotDriverPolicy )

BASE_LIBS_DECLARE_NODE_CLASS( TestRobotDriverPolicy )
{
	BASE_LIBS_DECLARE_NODE_CONSTRUCTOR( TestRobotDriverPolicy ){}

public:
	void spinFirst()
	{
		initAll();
	}
};

BASE_LIBS_INST_NODE( TestRobotDriverPolicyNode, "test_robot_driver_policy_node" )
