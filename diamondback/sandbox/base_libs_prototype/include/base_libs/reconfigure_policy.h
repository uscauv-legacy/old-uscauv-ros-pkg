#ifndef BASE_LIBS_BASE_LIBS_RECONFIGURE_POLICY_H_
#define BASE_LIBS_BASE_LIBS_RECONFIGURE_POLICY_H_

#include <base_libs/node_handle_policy.h>
#include <base_libs/param_reader.h>
#include <base_libs/generic_policy_adapter.h>
#include <base_libs/type_utils.h>
#include <dynamic_reconfigure/server.h>

namespace base_libs
{

#define POLICY_NAME Reconfigure

BASE_LIBS_DECLARE_POLICY( Reconfigure, NodeHandlePolicy )

template<class __ReconfigureType>
BASE_LIBS_DECLARE_POLICY_CLASS( Reconfigure )
{
	BASE_LIBS_MAKE_POLICY_NAME( Reconfigure )
	
public:
	typedef dynamic_reconfigure::Server<__ReconfigureType> _ReconfigureServer;
	typedef ReconfigurePolicy<__ReconfigureType> _ReconfigurePolicy;
	
protected:
	__ReconfigureType config_;

private:
	_ReconfigureServer * server_;
	typename _ReconfigureServer::CallbackType callback_;
	
public:
	BASE_LIBS_DECLARE_POLICY_CONSTRUCTOR( Reconfigure ),
		server_( NULL )
	{
		printPolicyActionStart( "create", this );
		printPolicyActionDone( "create", this );
	}
	
	BASE_LIBS_ENABLE_INIT
	{
		printPolicyActionStart( "initialize", this );
		
		server_ = new _ReconfigureServer(
			ros::NodeHandle(
				NodeHandlePolicy::nh_rel_,
				ros::ParamReader<std::string, 1>::readParam( 
					NodeHandlePolicy::nh_rel_, getMetaParamDef<std::string>( "reconfigure_namespace_name", "reconfigure_namespace", args... ), "reconfigure" ) ) );
		
		callback_ = boost::bind( &_ReconfigurePolicy::reconfigureCB_0, this, _1, _2 );
		server_->setCallback( callback_ );
		
		printPolicyActionDone( "initialize", this );
	}
	
	~ReconfigurePolicy()
	{
		if( server_ ) delete server_;
	}
	
private:
	void reconfigureCB_0( __ReconfigureType & config, uint32_t level )
	{
		config_ = config;
		reconfigureCB( config, level );
	}

protected:
	void reconfigureCB( __ReconfigureType & config, uint32_t level )
	{
		//
	}
};

#undef POLICY_NAME

}

#endif // BASE_LIBS_BASE_LIBS_RECONFIGURE_POLICY_H_
