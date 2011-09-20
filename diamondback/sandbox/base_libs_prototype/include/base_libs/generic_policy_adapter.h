#ifndef BASE_LIBS_BASE_LIBS_GENERIC_POLICY_ADAPTER_H_
#define BASE_LIBS_BASE_LIBS_GENERIC_POLICY_ADAPTER_H_

#include <base_libs/container.h>

namespace base_libs
{

template<class... __Policies>
class GenericPolicyAdapter : public __Policies...
{
public:
	// Construct a policy adapter that inherits from all specified
	// policies. Note that all policies in the inheritance group take
	// the same set of args during construction.
	template<class... __Args>
	GenericPolicyAdapter( __Args&&... args ) : __Policies( args )...
	{
		
	}
	
	// Do any post-construction initialization. Note that all policies
	// receive the same set of args.
	template<class... __Args>
	void init( __Args&&... args )
	{
		init<__Policies...>( args... );
	}
	
	// initialize the first policy in the subset
	// recurse through the remaining policies in the subset
	template<class __PoliciesSubset, class... __Args>
	void init( __Args&&... args )
	{
		typename ContainerTypes<__PoliciesSubset>::_Front::init( args... );
		init<typename ContainerTypes<__PoliciesSubset>::_Rest>( args... );
	}
	
	virtual void init(){}
};

}

#endif // BASE_LIBS_BASE_LIBS_GENERIC_POLICY_ADAPTER_H_
