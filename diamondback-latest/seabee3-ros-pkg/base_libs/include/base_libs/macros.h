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


// ########## Generic Policy Macros ####################################
// ---------------------------------------------------------------------
#define BASE_LIBS_DECLARE_POLICY( PolicyNameBase, __Policies... ) \
typedef base_libs::GenericPolicyAdapter< __Policies > _##PolicyNameBase##PolicyAdapterType;

// ---------------------------------------------------------------------
#define BASE_LIBS_DECLARE_POLICY_CLASS( PolicyNameBase ) \
class PolicyNameBase##Policy : public _##PolicyNameBase##PolicyAdapterType

// ---------------------------------------------------------------------
#define BASE_LIBS_MAKE_POLICY_NAME( PolicyNameBase ) \
public: const static inline std::string name(){ return #PolicyNameBase; }

// ---------------------------------------------------------------------
#define BASE_LIBS_DECLARE_POLICY_CONSTRUCTOR( PolicyNameBase ) \
public: \
	template<class... __Args> \
	PolicyNameBase##Policy( __Args&&... args ) \
	: \
		_##PolicyNameBase##PolicyAdapterType( args... )

// ########## Generic Node Macros ######################################
// ---------------------------------------------------------------------
#define BASE_LIBS_DECLARE_NODE( NodeNameBase, __Policies... ) \
typedef base_libs::Node< __Policies > _##NodeNameBase##NodeAdapterType;

// ---------------------------------------------------------------------
#define BASE_LIBS_DECLARE_NODE_CLASS( NodeNameBase ) \
class NodeNameBase##Node : public _##NodeNameBase##NodeAdapterType

// ---------------------------------------------------------------------
#define BASE_LIBS_DECLARE_NODE_CONSTRUCTOR( NodeNameBase ) \
public: \
	template<class... __Args> \
	NodeNameBase##Node( __Args&&... args ) \
	: \
		_##NodeNameBase##NodeAdapterType( args... )

// ########## Node Instantiation Macros ################################
// ---------------------------------------------------------------------
/// use: BASE_LIBS_INST_NODE( SomeNode, "some_node" )
#define BASE_LIBS_INST_NODE( NodeClassname, node_name_string ) \
int main( int argc, char ** argv ) \
{ \
	ros::init( argc, argv, node_name_string ); \
	ros::NodeHandle nh( "~" ); \
	\
	NodeClassname node_inst( nh ); \
	node_inst.spin(); \
	return 0; \
}

// ########## Generic Nodelet Macros ###################################
// ---------------------------------------------------------------------
#define BASE_LIBS_DECLARE_NODELET( namespace_name, ClassName ) \
namespace namespace_name { \
class ClassName##Nodelet : public base_libs::Nodelet<ClassName##Node>{}; }

// ########## Nodelet Instantiation Macros #############################
// ---------------------------------------------------------------------
#define BASE_LIBS_INST_NODELET( namespace_name, ClassName, nodelet_name ) \
PLUGINLIB_DECLARE_CLASS( namespace_name, nodelet_name, namespace_name::ClassName##Nodelet, nodelet::Nodelet )

// ########## Initable Policy Macros ###################################
// ---------------------------------------------------------------------
#define BASE_LIBS_ENABLE_INIT \
public: const static bool HAS_INIT_ = true; \
private: bool initialized_; \
private: inline void setInitialized( const bool & value ){ initialized_ = value; } \
public: template<class... __Args> \
void init( __Args&&... args )

// ---------------------------------------------------------------------
#define BASE_LIBS_CHECK_INITIALIZED \
if( !initialized_ ) PRINT_ERROR( "Policy [%s] has not been initialized!", name().c_str() ); \
if( !initialized_ ) PRINT_ERROR( "Some functionality may be disabled." )

// ---------------------------------------------------------------------
#define BASE_LIBS_SET_INITIALIZED \
this->setInitialized( true )

// ########## Updateable Policy Macros #################################
// ---------------------------------------------------------------------
#define BASE_LIBS_ENABLE_UPDATE \
const static bool HAS_UPDATE_ = true; \
template<class... __Args> \
void update( __Args&&... args )

// ########## Generic Callback Macros ##################################
// ---------------------------------------------------------------------
#define BASE_LIBS_DECLARE_MESSAGE_CALLBACK( callbackName, __MessageType ) \
void callbackName( const __MessageType::ConstPtr & msg )

// ---------------------------------------------------------------------
#define BASE_LIBS_DECLARE_CONDITIONAL_MESSAGE_CALLBACK( callbackName, __MessageType, condition ) \
typename std::enable_if<condition, void>::type \
callbackName( const __MessageType::ConstPtr & msg )

// ---------------------------------------------------------------------
#define BASE_LIBS_DECLARE_SERVICE_CALLBACK( callbackName, __ServiceType ) \
bool callbackName( __ServiceType::Request & request, __ServiceType::Response & response )

// ---------------------------------------------------------------------
#define BASE_LIBS_DECLARE_RECONFIGURE_CALLBACK( callbackName, __ReconfigureType ) \
void callbackName( __ReconfigureType & config, uint32_t level )

// ########## ImageProc Policy Macros ##################################
// ---------------------------------------------------------------------
#define IMAGE_PROC_PROCESS_IMAGE( image_ptr_name ) \
void processImage( cv_bridge::CvImageConstPtr & image_ptr_name )

// ########## Runable Policy Macros ####################################
// ---------------------------------------------------------------------
#define BASE_LIBS_SPIN_FIRST \
void spinFirst()

// ---------------------------------------------------------------------
#define BASE_LIBS_SPIN_ONCE \
void spinOnce()

// ########## Type Enable/Disable Macros ###############################
// ---------------------------------------------------------------------

#define BASE_LIBS_ENABLE_IF( __ReturnType, condition ) \
typename std::enable_if<(condition), __ReturnType>::type

// ---------------------------------------------------------------------
#define BASE_LIBS_ENABLE_IF_SAME( __ReturnType, __Type1, __Type2 ) \
BASE_LIBS_ENABLE_IF( __ReturnType, ( std::is_same<__Type1, __Type2>::value ) )
// "disable if not same" is an alias for "enable if same"
#define BASE_LIBS_DISABLE_IF_NOT_SAME BASE_LIBS_ENABLE_IF_SAME

// ---------------------------------------------------------------------
#define BASE_LIBS_DISABLE_IF_SAME( __ReturnType, __Type1, __Type2 ) \
BASE_LIBS_ENABLE_IF( __ReturnType, ( !std::is_same<__Type1, __Type2>::value ) )
// "enable_if_not_same" is an alias for "disable_if_same"
#define BASE_LIBS_ENABLE_IF_NOT_SAME BASE_LIBS_DISABLE_IF_SAME

// use pair: ENABLE_IF_SAME  / ENABLE_IF_NOT_SAME
// or:       DISABLE_IF_SAME / DISABLE_IF_NOT_SAME

// ########## Internal Macros ##########################################
// ---------------------------------------------------------------------
#define BASE_LIBS_DECLARE_INTERNAL_NAMESPACE \
namespace base_libs

// ---------------------------------------------------------------------
#define __BASE_LIBS_FUNCTION_TYPE \
std::function

#endif // BASE_LIBS_BASE_LIBS_MACROS_H_
