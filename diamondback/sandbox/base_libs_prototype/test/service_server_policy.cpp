#include <base_libs/node.h>
#include <base_libs/service_server_policy.h>
#include <base_libs_prototype/TestService1.h>
#include <base_libs_prototype/TestService2.h>

typedef base_libs_prototype::TestService1 _TestService1;
typedef base_libs::ServiceServerPolicy<_TestService1> _TestService1ServerPolicy;
typedef base_libs_prototype::TestService2 _TestService2;
typedef base_libs::ServiceServerPolicy<_TestService2> _TestService2ServerPolicy;

BASE_LIBS_DECLARE_NODE( TestServiceServerPolicy, _TestService1ServerPolicy, _TestService2ServerPolicy )

BASE_LIBS_DECLARE_NODE_CLASS( TestServiceServerPolicy )
{
	BASE_LIBS_DECLARE_NODE_CONSTRUCTOR( TestServiceServerPolicy ){}

public:
	void spinFirst()
	{
		//initAll();
		
		_TestService1ServerPolicy::init( "service_name_param", std::string( "service1_name" ) );
		_TestService1ServerPolicy::registerCallback( base_libs::auto_bind( &TestServiceServerPolicyNode::service1CB, this ) );
		
		_TestService2ServerPolicy::init( "service_name_param", std::string( "service2_name" ) );
		_TestService2ServerPolicy::registerCallback( base_libs::auto_bind( &TestServiceServerPolicyNode::service2CB, this ) );
	}
	
	bool service1CB( _TestService1::Request & request, _TestService1::Response & response )
	{
		PRINT_INFO( "Got service1CB" );
		return true;
	}
	
	bool service2CB( _TestService2::Request & request, _TestService2::Response & response )
	{
		PRINT_INFO( "Got service2CB" );
		return true;
	}
};

BASE_LIBS_INST_NODE( TestServiceServerPolicyNode, "test_service_server_policy_node" )
