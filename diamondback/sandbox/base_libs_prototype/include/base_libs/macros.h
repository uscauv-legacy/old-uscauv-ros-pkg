#ifndef BASE_LIBS_BASE_LIBS_MACROS_H_
#define BASE_LIBS_BASE_LIBS_MACROS_H_

#include <ros/ros.h>

// use: BASE_LIBS_DECLARE_NODE( SomeNode, "some_node" )
#define BASE_LIBS_DECLARE_NODE( __Class, node_name_string ) \
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

#endif // BASE_LIBS_BASE_LIBS_MACROS_H_
