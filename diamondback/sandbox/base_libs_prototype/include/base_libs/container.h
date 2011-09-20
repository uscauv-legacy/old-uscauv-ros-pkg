#ifndef BASE_LIBS_BASE_LIBS_CONTAINER_H_
#define BASE_LIBS_BASE_LIBS_CONTAINER_H_

#include <type_traits>

template<bool __Enable__, class... __Types>
struct DeclareIfSingleTypeHelper
{
	typedef void _Type;
};

template<class __Type>
struct DeclareIfSingleTypeHelper<true, __Type>
{
	typedef __Type _Type;
};

// typedefs _Type if __Types has only one element
template<class... __Types>
struct DeclareIfSingleType
{
	typedef typename DeclareIfSingleTypeHelper<sizeof...( __Types ) == 1, __Types...>::_Type _Type;
};

// stores a list of types; provides tools for accessing that list of types
template<class... __Types>
struct Container
{
	const static unsigned int num_types_ = sizeof...( __Types );
	
	// typedef for the front type
	typedef typename DeclareIfSingleType<__Types...>::_Type _Type;
	// front() will always return a container of with a __Types... of size 1
	
	//typedef typename ContainerTypes<__Types...>::_Front _Front;
	//typedef typename ContainerTypes<__Types...>::_Rest _Rest;
	
	//typedef Container<__Types...> _Container;
	//typedef typename ContainerTypes<_Container>::_Front _Front;
	//typedef typename ContainerTypes<_Container>::_Rest _Rest;
};

// can't use qualified-id in decltype ie decltype(std::vector<int>)::value_type until gcc 4.6
// work-around is to make several helper classes, in this case:
// - ContainerTypesHelper
// - ContainerTypes
struct ContainerTypesHelper
{
	// get a container for the type at the front of the container
	template<class __Front, class... __Rest>
	static Container<__Front> front( const Container<__Front, __Rest...> & container )
	{
		return Container<__Front>();
	}
	
	static Container<void> front( const Container<> & container )
	{
		return Container<void>();
	}
	
	/*static auto front() -> decltype( frontHelper( __Container() ) )
	{
		return frontHelper( __Container() );
	}*/

	// get a container for the types after the front of the container
	template<class __Front, class... __Rest>
	static Container<__Rest...> rest( const Container<__Front, __Rest...> & container )
	{
		return Container<__Rest...>();
	}
	
	static Container<void> rest( const Container<> & container )
	{
		return Container<void>();
	}
	
	/*static auto rest() -> decltype( restHelper( __Container() ) )
	{
		return restHelper( __Container() );
	}*/
	
//	typedef decltype( front( Container<__Types...>() ) ) _Front;
//	typedef decltype( rest( Container<__Types...>() ) ) _Rest;
};

template<class __Container>
struct ContainerTypes
{
	typedef decltype( ContainerTypesHelper::front( __Container() ) ) _FrontContainer;
	typedef decltype( ContainerTypesHelper::rest( __Container() ) ) _RestContainer;
	typedef typename _FrontContainer::_Type _Type;
	typedef _Type _Front;
	typedef _RestContainer _Rest;
};

#endif // BASE_LIBS_BASE_LIBS_CONTAINER_H_
