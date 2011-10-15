#include <base_libs/type_utils.h>
#include <string>
#include <stdio.h>
#include <iostream>

#define TEST_ARGS \
"reconfigure_namespace", double( 0.5 ), \
"reconfigure_namespace", std::string( "hello" ), \
"reconfigure_namespace", float( 0.3 ), \
"reconfigure_namespace2", std::string( "hello2" ), \
"reconfigure_namespace3", std::string( "hello3" ) \

template<class __Data>
void print( const __Data & data )
{
	std::cout << data << std::endl;
}

struct TYPE_DNE{};

int main( int argc, char ** argv )
{
	// getFirstOfType takes an output type and a list of args
	// if the output type is found in the list of args; return the first arg with matching type
	// else fail at compile time
	
	// fake type will fail at compile time
	//print( base_libs::getFirstOfType<TYPE_DNE>( TEST_ARGS ) );
	
	print( base_libs::getFirstOfType<double>( TEST_ARGS ) );
	print( base_libs::getFirstOfType<float>( TEST_ARGS ) );
	print( base_libs::getFirstOfType<std::string>( TEST_ARGS ) );
	
	printf( "-----\n" );
	
	// getMetaParam takes an output type, a key, and a list of key-value pairs
	// if the output type is not found in the list of key-value pairs; fail at compile time
	// else if the key is not found in the list of key-value pairs, give a warning at runtime and return output_type()
	// else return the requested value
	
	// getMetaParamDef is identical to getMetaParam except it also takes a required default value, which it returns instead of output_type()
	
	// notice that we can store multiple values under the same key
	// if a complete duplicate (key and value) is given in the list, the first copy encountered is returned
	
	// using a fake type will fail at compile time
	//base_libs::getMetaParam<TYPE_DNE>( "reconfigure_namespace" ), TEST_ARGS );
	
	// get the param with type double and key reconfigure_namespace
	print( base_libs::getMetaParam<double>( "reconfigure_namespace", TEST_ARGS ) );
	
	// get the param with type std::string and key reconfigure_namespace	
	print( base_libs::getMetaParam<std::string>( "reconfigure_namespace", TEST_ARGS ) );
	
	// get the param with type float and key reconfigure_namespace
	print( base_libs::getMetaParam<float>( "reconfigure_namespace", TEST_ARGS ) );
	
	// get the param with type std::string and key reconfigure_namespace2
	print( base_libs::getMetaParam<std::string>( "reconfigure_namespace2", TEST_ARGS ) );
	
	// get the param with type std::string and key reconfigure_namespace2
	print( base_libs::getMetaParam<std::string>( "reconfigure_namespace3", TEST_ARGS ) );
	
	// using a fake namespace will fail at runtime
	print( base_libs::getMetaParam<std::string>( "reconfigure_namespace_DNE", TEST_ARGS ) );
	
	// using a fake namespace will fail at runtime
	print( base_libs::getMetaParam<float>( "reconfigure_namespace_DNE2", TEST_ARGS ) );
	
	// using a fake namespace will fail at runtime
	print( base_libs::getMetaParam<double>( "reconfigure_namespace_DNE3", TEST_ARGS ) );
	
	// using a fake namespace will fail at runtime; provide default
	print( base_libs::getMetaParamDef<std::string>( "reconfigure_namespace_DNE", "default1", TEST_ARGS ) );
	
	// using a fake namespace will fail at runtime; provide default
	print( base_libs::getMetaParamDef<float>( "reconfigure_namespace_DNE2", 0.1, TEST_ARGS ) );
	
	// using a fake namespace will fail at runtime; provide default
	print( base_libs::getMetaParamDef<double>( "reconfigure_namespace_DNE3", 0.2, TEST_ARGS ) );
	
	return 0;
}
