// for Nodelet
#include <base_libs/nodelet.h>

// for our custom node
#include <base_libs_demo/base_libs_demo.h>

// "Declare" a nodelet called base_libs_demo::BaseLibsDemo
// nodelet is designed to wrap an existing node; here, we want to wrap BaseLibsDemoNode
// This macro takes two arguments: the namespace of the new class ( @namespace ) and the base name of the new class ( @base_name )
// This macro creates a new class with public inheritance from a nodelet that wraps the corresponding node:
// 
// @namespace::@base_name##Nodelet : public base_libs::Nodelet< @base_name##Node >
// 
// In this case, the result is:
// 
// base_libs_demo::BaseLibsDemoNodelet : public base_libs::Nodelet< BaseLibsDemoNode >
BASE_LIBS_DECLARE_NODELET( base_libs_demo, BaseLibsDemo )

// wrapper around PLUGINLIB_DECLARE_CLASS
// we want to register base_lib_demo::BaseLibsDemoNodelet and call it demo
// This macro takes three arguments: the namespace of the nodelet ( @namespace ), the base name of the nodelet class ( @base_name ), and the name of the registered nodelet ( @name )
// This macro just calls PLUGINLIB_DECLARE_CLASS with the appropriate information:
// 
// PLUGINLIB_DECLARE_CLASS( @namespace, @name, @namespace::@base_name##Nodelet, nodelet::Nodelet )
//
// In this case, the result is:
//
// PLUGINLIB_DECLARE_CLASS( base_libs_demo, demo, base_libs_demo::BaseLibsDemoNodelet, nodelet::Nodelet )
BASE_LIBS_INST_NODELET( base_libs_demo, BaseLibsDemo, demo )
