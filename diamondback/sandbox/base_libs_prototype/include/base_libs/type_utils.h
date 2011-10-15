#ifndef BASE_LIBS_BASE_LIBS_TYPE_UTILS_H_
#define BASE_LIBS_BASE_LIBS_TYPE_UTILS_H_

#include <type_traits>
#include <string>
#include <stdlib.h>
#include <base_libs/console.h>

namespace base_libs
{
	struct TYPE_NOT_FOUND{};
	
	template<class __Desired>
	static __Desired getFirstOfType(){ return TYPE_NOT_FOUND(); }
	
	// matching value found; return it
	template<class __Desired, class __Current, class... __Rest>
	static typename std::enable_if<std::is_same<__Desired, __Current>::value, __Desired>::type
	getFirstOfType( __Current & current, __Rest&&... rest )
	{
		return current;
	}
	
	// iterate through types in __Rest until a matching type is found
	// this will fail at compile time if no matching type exists
	template<class __Desired, class __Current, class... __Rest>
	static typename std::enable_if<!std::is_same<__Desired, __Current>::value, __Desired>::type
	getFirstOfType( __Current & current, __Rest&&... rest )
	{
		return getFirstOfType<__Desired>( rest... );
	}
	
	template<class __Desired>
	__Desired getMetaParamRec( const std::string & name, const __Desired & default_value )
	{
		PRINT_WARN( "Failed to find key [%s]", name.c_str() );
		return default_value;
		// fail at runtime
		//abort();
	}
	
	// ####
	
	template<class __Desired, class __Current, class... __Rest>
	static typename std::enable_if<(!std::is_same<__Desired, __Current>::value), __Desired>::type
	getMetaParamRec( const std::string & name, const __Desired & default_value, const std::string & current_name, __Current & current, __Rest&&... rest );
	
	template<class __Desired, class __Current, class... __Rest>
	static typename std::enable_if<(std::is_same<__Desired, __Current>::value), __Desired>::type
	getMetaParamRec( const std::string & name, const __Desired & default_value, const std::string & current_name, __Current & current, __Rest&&... rest );
	
	// ####
	
	template<class __Desired, class __Current, class... __Rest>
	static typename std::enable_if<(!std::is_same<__Desired, __Current>::value), __Desired>::type
	getMetaParamRec( const std::string & name, const __Desired & default_value, const std::string & current_name, __Current & current, __Rest&&... rest )
	{
		return getMetaParamRec<__Desired>( name, default_value, rest... );
	}
	
	template<class __Desired, class __Current, class... __Rest>
	static typename std::enable_if<(std::is_same<__Desired, __Current>::value), __Desired>::type
	getMetaParamRec( const std::string & name, const __Desired & default_value, const std::string & current_name, __Current & current, __Rest&&... rest )
	{
		PRINT_INFO( "Found key [%s]", name.c_str() );
		return name == current_name ? current : getMetaParamRec<__Desired>( name, default_value, rest... );
	}
	
	// ####
	
	template<class __Desired, class... __Rest>
	static __Desired getMetaParam( const std::string & name, __Rest&&... rest )
	{
		// make sure desired type exists in list; otherwise fail at compile time
		getFirstOfType<__Desired>( rest... );
		return getMetaParamRec<__Desired>( name, __Desired(), rest... );
	}
	
	template<class __Desired, class... __Rest>
	static __Desired getMetaParamDef( const std::string & name, const __Desired & default_value, __Rest&&... rest )
	{
		return getMetaParamRec<__Desired>( name, default_value, rest... );
	}
}

#endif // BASE_LIBS_BASE_LIBS_TYPE_UTILS_H_
