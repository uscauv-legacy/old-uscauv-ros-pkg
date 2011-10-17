#include <base_libs/nodelet.h>
#include <base_libs_demo/base_libs_demo.h>

// nodelet is designed to wrap an existing node; here, we want to wrap BaseLibsDemoNode
// a new class, image_server_prototype::ImageServerNodelet, is created for this purpose
BASE_LIBS_DECLARE_NODELET( base_libs_demo, BaseLibsDemo )

// wrapper around PLUGINLIB_DECLARE_CLASS
// we want to register base_lib_demo::BaseLibsDemoNodelet and call it demo
BASE_LIBS_INST_NODELET( base_libs_demo, BaseLibsDemo, demo )
