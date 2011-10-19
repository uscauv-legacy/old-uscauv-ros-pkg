// <PACKAGE_NAME>_<_INCLUDE_SUBDIR>_<FILE_NAME>_H_
#ifndef BASE_LIBS_DEMO_BASE_LIBS_DEMO_BASE_LIBS_DEMO_H_
#define BASE_LIBS_DEMO_BASE_LIBS_DEMO_BASE_LIBS_DEMO_H_

// for Node
#include <base_libs/node.h>

// if we wanted any policies, we'd include them here

// "Declare" a node called BaseLibsDemo
// The first argument ( @base_name ) for this macro is the base name of the node you want to create. The real name will have "Node" appended to it.
// The rest of the arguments ( @policies ) are the policies you want this node to use.
// This macro declares a new type:
// 
// typedef base_libs::Node< @policies > _##@base_name##NodeAdapterType;
// 
// In this case, since @policies is empty, the result is:
// 
// typedef base_libs::Node<> _BaseLibsDemoNodeAdapterType
BASE_LIBS_DECLARE_NODE( BaseLibsDemo )

// "Declare" the node class called BaseLibsDemo
// This is where the class is actually created
// The only argument to this macro is a base name ( @base_name )
// This macro declares a new class with public inheritance from the adapter type we made with the BASE_LIBS_DECLARE_NODE macro:
// 
// class @base_name##Node : public _##@base_name##NodeAdapterType
// 
// In this case, the result is:
// 
// class BaseLibsDemoNode : public _BaseLibsDemoNodeAdapterType
// 
// Note that this still allows us to inherit from other classes as well, if necessary, via:
// BASE_LIBS_DECLARE_NODE_CLASS( whatever ), public PubClass, private PrivClass, etc
BASE_LIBS_DECLARE_NODE_CLASS( BaseLibsDemo )
{
	// "Declare" the constructor for BaseLibdDemoNode
	// The only argument to this macro is a base name ( @base_name )
	// This macro declares a constructor with public inheritance from the adapter type we made earlier: 
	//
	//  public:
	//    template<class... __Args>
	//    @base_name##Node( __Args&&... args ) : public _##@base_name##NodeAdapterType( args... )
	//
	// In this case, the result is:
	//
	//  public:
	//    template<class... __Args>
	//    BaseLibsDemoNode( __Args&&... args ) : public _BaseLibsDemoNodeAdapterType( args... )
	//
	// Note that this still allows us to construct other classes and members, via:
	// BASE_LIBS_DECLARE_NODE_CLASS( whatever ), PubClass( some_value ), PrivClass( some_value1 ), member1_( some_value2 ), etc
	BASE_LIBS_DECLARE_NODE_CONSTRUCTOR( BaseLibsDemo )
	{
		//
	}
	
	// overridden from RunablePolicy
	// called within spin() just before the first call to spinOnce()
	void spinFirst()
	{
		initAll();
	}
	
	// overridden from RunablePolicy
	// called within a loop in spin()
	// RunablePolicy will automatically update the ROS event queue and sleep
	// for the appropriate time ( see RunablePolicy::loop_rate_ ) after calling
	// this function.
	void spinOnce()
	{
		// PRINT_INFO is a macro similar to ROS_INFO and functions like printf()
		PRINT_INFO( "Spinning!" );
	}
};

#endif // BASE_LIBS_DEMO_BASE_LIBS_DEMO_BASE_LIBS_DEMO_H_
