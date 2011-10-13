#ifndef BASE_LIBS_BASE_LIBS_GENERIC_POLICY_ADAPTER_H_
#define BASE_LIBS_BASE_LIBS_GENERIC_POLICY_ADAPTER_H_

#include <base_libs/macros.h>
#include <base_libs/container.h>
#include <utility>

namespace base_libs
{

template<class... __Policies>
class GenericPolicyAdapter : public __Policies...
{
public:
	// Construct a policy adapter that inherits from all specified
	// policies. Note that all policies in the inheritance group take
	// the same set of args during construction.
	
	// gcc can't compile this; "invalid use of pack expansion expression"
	// see http://stackoverflow.com/questions/7694201/unpacking-parameter-pack-of-args-into-the-constructor-of-each-class-defined-in-a
	//template<class... __Args>
	//GenericPolicyAdapter( __Args... args ) : __Policies( args... )
	
	template<class __Arg>
	GenericPolicyAdapter( __Arg & arg ) : __Policies( arg )...
	{
		
	}
	
	// for all non-initable policies
	template<class __Policy, class... __Args>
	typename std::enable_if<( !__Policy::HAS_INIT_ ), void>::type
	tryInit( __Args&&... args ) {}
	
	// for all initable policies
	template<class __Policy, class... __Args>
	typename std::enable_if<( __Policy::HAS_INIT_ ), void>::type
	tryInit( __Args&&... args )
	{
		__Policy::init( args... );
	}
	
	// initialize the first policy in the subset
	// recurse through the remaining policies in the subset
	template<class __PoliciesSubset, class... __Args>
	typename std::enable_if<(__PoliciesSubset::num_types_ > 0), void>::type
	initRec( __Args&&... args )
	{
		tryInit<typename ContainerTypes<__PoliciesSubset>::_Front>( args... );
		initRec<typename ContainerTypes<__PoliciesSubset>::_Rest>( args... );
	}
	
	template<class __PoliciesSubset, class... __Args>
	typename std::enable_if<(__PoliciesSubset::num_types_ == 0), void>::type
	initRec( __Args&&... args ) {}
	
	// Do any post-construction initialization. Note that all policies
	// receive the same set of args.
	template<class... __Args>
	void initAll( __Args&&... args )
	{
		initRec<Container<__Policies...> >( args... );
	}
	
	BASE_LIBS_ENABLE_INIT{}
};

}

#endif // BASE_LIBS_BASE_LIBS_GENERIC_POLICY_ADAPTER_H_
