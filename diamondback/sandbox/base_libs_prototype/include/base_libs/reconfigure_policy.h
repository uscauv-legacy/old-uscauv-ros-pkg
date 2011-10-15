#ifndef BASE_LIBS_BASE_LIBS_RECONFIGURE_POLICY_H_
#define BASE_LIBS_BASE_LIBS_RECONFIGURE_POLICY_H_

#include <base_libs/node_handle_policy.h>
#include <base_libs/auto_bind.h>
#include <dynamic_reconfigure/server.h>

namespace base_libs
{

BASE_LIBS_DECLARE_POLICY( Reconfigure, NodeHandlePolicy )

template<class __ReconfigureType>
BASE_LIBS_DECLARE_POLICY_CLASS( Reconfigure )
{
	BASE_LIBS_MAKE_POLICY_NAME( Reconfigure )
	
public:
	typedef __ReconfigureType _ReconfigureType;
	typedef dynamic_reconfigure::Server<__ReconfigureType> _ReconfigureServer;
	typedef ReconfigurePolicy<__ReconfigureType> _ReconfigurePolicy;
	
protected:
	__ReconfigureType config_;

private:
	_ReconfigureServer * server_;
	typename _ReconfigureServer::CallbackType external_callback_;
	
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
					NodeHandlePolicy::nh_rel_,
					getMetaParamDef<std::string>( "reconfigure_namespace_name", "reconfigure_namespace", args... ),
					"reconfigure" ) ) );
		
		server_->setCallback( base_libs::auto_bind( &_ReconfigurePolicy::reconfigureCB_0, this ) );
		
		printPolicyActionDone( "initialize", this );
	}
	
	~ReconfigurePolicy()
	{
		if( server_ ) delete server_;
	}
	
	void registerCallback( typename _ReconfigureServer::CallbackType external_callback )
	{
		external_callback_ = external_callback;
	}
	
private:
	void reconfigureCB_0( __ReconfigureType & config, uint32_t level )
	{
		config_ = config;
		if( external_callback_ ) external_callback_( config, level );
	}

/*
protected:
	void reconfigureCB( __ReconfigureType & config, uint32_t level )
	{
		//
	}*/
};

}

#endif // BASE_LIBS_BASE_LIBS_RECONFIGURE_POLICY_H_
