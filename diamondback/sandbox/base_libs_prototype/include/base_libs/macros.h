/***************************************************************************
 *  include/base_libs/macros.h
 *  --------------------
 * 
 *  Copyright (c) 2011, Edward T. Kaszubski ( ekaszubski@gmail.com )
 *  All rights reserved.
 * 
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are
 *  met:
 *  
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following disclaimer
 *    in the documentation and/or other materials provided with the
 *    distribution.
 *  * Neither the name of seabee3-ros-pkg nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *  
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 **************************************************************************/

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

#define BASE_LIBS_DECLARE_MESSAGE_CALLBACK( callback_name, message_type ) \
void callback_name( const message_type::ConstPtr & msg )

#define BASE_LIBS_ENABLE_INIT \
public: const static bool HAS_INIT_ = true; \
private: bool initialized_; \
private: inline void setInitialized( const bool & value ){ initialized_ = value; } \
public: template<class... __Args> \
void init( __Args&&... args )

#define BASE_LIBS_CHECK_INITIALIZED \
if( !initialized_ ) PRINT_ERROR( "Policy [%s] has not been initialized!", name().c_str() ); \
if( !initialized_ ) PRINT_ERROR( "Some functionality may be disabled." )

#define BASE_LIBS_SET_INITIALIZED \
this->setInitialized( true )

#define BASE_LIBS_ENABLE_UPDATE \
const static bool HAS_UPDATE_ = true; \
template<class... __Args> \
void update( __Args&&... args )
		
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
