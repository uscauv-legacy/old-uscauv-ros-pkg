#include <base_libs/node.h>
#include <base_libs/reconfigure_policy.h>
#include <base_libs_prototype/TestConfig.h>
#include <base_libs_prototype/Test2Config.h>

typedef base_libs::ReconfigurePolicy<base_libs_prototype::TestConfig> _TestConfigReconfigurePolicy;
typedef base_libs::ReconfigurePolicy<base_libs_prototype::Test2Config> _Test2ConfigReconfigurePolicy;

BASE_LIBS_DECLARE_NODE( TestReconfigurePolicy, _TestConfigReconfigurePolicy, _Test2ConfigReconfigurePolicy )

BASE_LIBS_DECLARE_NODE_CLASS( TestReconfigurePolicy )
{
	BASE_LIBS_DECLARE_NODE_CONSTRUCTOR( TestReconfigurePolicy )
	{
		//
	}
	
	void spinFirst()
	{
		// call init() on all initable policies
		//initAll();
		
		// call init( ... ) on all initable policies
		//initAll( "reconfigure_namespace_name", std::string( "reconfigure_namespace_1" ) );
		
		// call init on just _TestConfigReconfigurePolicy
		// All ReconfigurePolicy instances use getMetaParamDef to look up the key "reconfigure_namespace_name" in the list of init() params
		// and default to "reconfigure_namspace" if no matching key is found. This value is then fed into a ParamReader as the name of a param.
		// The value of that param is then used as the reconfigure namespace.
		//
		// So, when the following line is called, a ParamReader will try to find "node_name/reconfigure_namespace_1" or default to "reconfigure" (let's call the result <reconfigure_ns>
		// Then a dynamic reconfigure server will be set up as: node_name/<reconfigure_ns>
		// If this function was given no arguments, the ParamReader would instead try to find "node_name/reconfigure_namespace" (since the default value for they meta-param key
		// "reconfigure_namespace_name", as defined in ReconfigurePolicy, is "reconfigure_namespace") or default to "reconfigure", etc
		_TestConfigReconfigurePolicy::init( "reconfigure_namespace_name", std::string( "reconfigure_namespace_1" ) );
		_TestConfigReconfigurePolicy::registerCallback( base_libs::auto_bind( &TestReconfigurePolicyNode::reconfigureCB1, this ) );
		
		// call init on just _Test2ConfigReconfigurePolicy
		_Test2ConfigReconfigurePolicy::init( "reconfigure_namespace_name", std::string( "reconfigure_namespace_2" ) );
		_Test2ConfigReconfigurePolicy::registerCallback( base_libs::auto_bind( &TestReconfigurePolicyNode::reconfigureCB2, this ) );
		
		// so now, when we run this node using the above inits, we can do:
		// ./node_name _reconfigure_namespace_1:=reconfigure1 _reconfigure_namespace_2:=reconfigure2
		// and our two dynamic reconfigure servers will work in harmony; the server for TestConfig
		// will show up on /node_name/reconfigure1 and the server for TestConfig2 will show up on
		// /node_name/reconfigure2
		
		//_TestConfigReconfigurePolicy::registerCallback( std::auto_bind<
	}
	
	void spinOnce()
	{
		std::cout << "." << std::flush;
	}
	
	void reconfigureCB1( _TestConfigReconfigurePolicy::_ReconfigureType & config, uint32_t level )
	{
		printf( "Got reconfigureCB1\n" );
	}
	
	void reconfigureCB2( _Test2ConfigReconfigurePolicy::_ReconfigureType & config, uint32_t level )
	{
		printf( "Got reconfigureCB2\n" );
	}
};

BASE_LIBS_INST_NODE( TestReconfigurePolicyNode, "test_reconfigure_policy_node" )
