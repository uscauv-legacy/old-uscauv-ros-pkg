#ifndef BASE_LIBS_BASE_LIBS_NODE_H_
#define BASE_LIBS_BASE_LIBS_NODE_H_

#include <base_libs/runable_policy.h>

namespace base_libs
{

// wrap GenericPolicyAdapter and add RunablePolicy to the front of __Policies...
template<class... __Policies>
class Node : public GenericPolicyAdapter<RunablePolicy, __Policies...>
{
public:
	template<class... __Args>
	Node( __Args&&... args ) : GenericPolicyAdapter<RunablePolicy, __Policies...>( args... )
	{
		
	}
	
	// for all non-initable policies
	template<class __Policy, class... __Args>
	typename std::enable_if<( !__Policy::HAS_UPDATE_ ), void>::type
	tryUpdate( __Args&&... args ) {}
	
	// for all initable policies
	template<class __Policy, class... __Args>
	typename std::enable_if<( __Policy::HAS_UPDATE_ ), void>::type
	tryUpdate( __Args&&... args )
	{
		__Policy::update( args... );
	}
	
	// initialize the first policy in the subset
	// recurse through the remaining policies in the subset
	template<class __PoliciesSubset, class... __Args>
	typename std::enable_if<(__PoliciesSubset::num_types_ > 0), void>::type
	updateRec( __Args&&... args )
	{
		tryUpdate<typename ContainerTypes<__PoliciesSubset>::_Front>( args... );
		updateRec<typename ContainerTypes<__PoliciesSubset>::_Rest>( args... );
	}
	
	template<class __PoliciesSubset, class... __Args>
	typename std::enable_if<(__PoliciesSubset::num_types_ == 0), void>::type
	updateRec( __Args&&... args ) {}
	
	// Do any post-construction initialization. Note that all policies
	// receive the same set of args.
	template<class... __Args>
	void updateAll( __Args&&... args )
	{
		updateRec<Container<__Policies...> >( args... );
	}
	
	BASE_LIBS_ENABLE_UPDATE{}
};

}

#endif // BASE_LIBS_BASE_LIBS_NODE_H_
