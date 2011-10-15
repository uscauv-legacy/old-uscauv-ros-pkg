#ifndef BASE_LIBS_BASE_LIBS_POLICY_H_
#define BASE_LIBS_BASE_LIBS_POLICY_H_

// for PRINT_INFO, etc
#include <base_libs/console.h>
#include <base_libs/generic_policy_adapter.h>
#include <base_libs/type_utils.h>

namespace base_libs
{

class Policy
{
public:
	const static inline std::string name(){ return "Base"; }
	
	template<class... __Args>
	Policy( __Args&&... args )
	{
		// discard args... but don't fail to compile if anything is passed in
		PRINT_INFO( "--------------------" );
		printPolicyActionStart( "create", this );
		printPolicyActionDone( "create", this );
	}
	
	template<class __Policy>
	static void printPolicyAction( std::string action, __Policy * policy )
	{
		PRINT_INFO( "[%s] on policy: [%s]", action.c_str(), __Policy::name().c_str() );
	}
	
	template<class __Policy>
	static void printPolicyActionStart( std::string action, __Policy * policy )
	{
		printPolicyAction( "Start: " + action, policy );
	}
	
	template<class __Policy>
	static void printPolicyActionDone( std::string action, __Policy * policy )
	{
		printPolicyAction( "Done: " + action, policy );
	}
};

}

#endif // BASE_LIBS_BASE_LIBS_POLICY_H_
