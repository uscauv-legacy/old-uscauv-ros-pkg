#ifndef BASE_LIBS_BASE_LIBS_MACROS_H_
#define BASE_LIBS_BASE_LIBS_MACROS_H_

//#include <ros/ros.h>

//#define BASE_LIBS_DECLARE_HEADER( package_name, include_name, header_name )
//'#ifndef package_name##_##include_name##_##header_name##_H_'
//'#define package_name##_##include_name##_##header_name##_H_'

// use: BASE_LIBS_INST_NODE( SomeNode, "some_node" )
#define BASE_LIBS_INST_NODE( __Class, node_name_string ) \
int main( int argc, char ** argv ) \
{ \
	ros::init( argc, argv, node_name_string ); \
	ros::NodeHandle nh( "~" ); \
	\
	__Class class_inst( nh ); \
	class_inst.spin(); \
	return 0; \
}

#define IMAGE_PROC_PROCESS_IMAGE( image_ptr_name ) \
void processImage( cv_bridge::CvImageConstPtr & image_ptr_name )

#define BASE_LIBS_DECLARE_STANDARD_CALLBACK( callback_name, message_type ) \
void callback_name( const message_type::ConstPtr & msg )

#define BASE_LIBS_ENABLE_INIT \
const static bool HAS_INIT_ = true; \
template<class... __Args> \
void init( __Args&&... args )

#define BASE_LIBS_ENABLE_UPDATE \
const static bool HAS_UPDATE_ = true; \
template<class... __Args> \
void update( __Args&&... args )

/*#define BASE_LIBS_DECLARE_POLICY( __NameBase, __Policies... ) \
class __NameBase##Policy : public GenericPolicyAdapter< __Policies > \
{ \
private: \
	typedef GenericPolicyAdapter< __Policies > _GenericPolicyAdapter; \
 \
public: \
	const static inline std::string name(){ return #__NameBase; } \
	 \
	template<class... __Args> \
	__NameBase##Policy( __Args&&... args ) \
	: \
		_GenericPolicyAdapter( args... )*/
		
#define BASE_LIBS_DECLARE_POLICY( __NameBase, __Policies... ) \
typedef base_libs::GenericPolicyAdapter< __Policies > _##__NameBase##PolicyAdapterType;

#define BASE_LIBS_DECLARE_POLICY_CLASS( __NameBase ) \
class __NameBase##Policy : public _##__NameBase##PolicyAdapterType

#define BASE_LIBS_MAKE_POLICY_NAME( __NameBase ) \
public: const static inline std::string name(){ return #__NameBase; }

#define BASE_LIBS_DECLARE_POLICY_CONSTRUCTOR( __NameBase ) \
public: \
	template<class... __Args> \
	__NameBase##Policy( __Args&&... args ) \
	: \
		_##__NameBase##PolicyAdapterType( args... )
		
#define BASE_LIBS_DECLARE_NODE( __NameBase, __Policies... ) \
typedef base_libs::Node< __Policies > _##__NameBase##NodeAdapterType;

#define BASE_LIBS_DECLARE_NODE_CLASS( __NameBase ) \
class __NameBase##Node : public _##__NameBase##NodeAdapterType

#define BASE_LIBS_DECLARE_NODE_CONSTRUCTOR( __NameBase ) \
public: \
	template<class... __Args> \
	__NameBase##Node( __Args&&... args ) \
	: \
		_##__NameBase##NodeAdapterType( args... )

#define BASE_LIBS_DECLARE_NODELET( namespace_name, class_name ) \
namespace namespace_name { \
class class_name##Nodelet : public base_libs::Nodelet<class_name##Node>{}; }

#define BASE_LIBS_INST_NODELET( namespace_name, class_name, nodelet_name ) \
PLUGINLIB_DECLARE_CLASS( namespace_name, nodelet_name, namespace_name::class_name##Nodelet, nodelet::Nodelet )

#endif // BASE_LIBS_BASE_LIBS_MACROS_H_
