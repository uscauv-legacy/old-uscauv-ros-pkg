#ifndef BASE_LIBS_BASE_LIBS_MULTITYPED_LINKED_LIST_H_
#define BASE_LIBS_BASE_LIBS_MULTITYPED_LINKED_LIST_H_

#include <base_libs/container.h>

// 2 or more data types
template<class __Data, class __RestContainer>
class MultitypedNode
{
public:
	typedef typename ContainerTypes<__RestContainer>::_Rest _ChildRestContainer;
	typedef typename ContainerTypes<__RestContainer>::_Front _ChildData;
	
	typedef MultitypedNode<_ChildData, _ChildRestContainer> _ChildMultitypedNode;
	typedef __Data _Data;
	
	__Data data_;
	_ChildMultitypedNode * child_;
	const static bool has_child_ = true;
	
	MultitypedNode() : child_( NULL ) {}
	
	template<class... __RestChildData>
	MultitypedNode( const __Data & data, __RestChildData&&... rest_child_data ) : data_( data ), child_( new _ChildMultitypedNode( rest_child_data... ) )
	{
		//printf( "constructing node with data type %s and %lu children\n", typeid( __Data ).name(), sizeof...( __RestChildData ) );
		//printf( "child exists: %u\n", child_ != false );
	}
};

// one data type
template<class __Data>
class MultitypedNode<__Data, Container<> >
{
public:
	typedef Container<__Data> _Container;
	typedef __Data _Data;
	
	__Data data_;
	const static bool has_child_ = false;
	
	MultitypedNode(){}
	
	MultitypedNode( const __Data & data ) : data_( data )
	{
		//printf( "constructing node with data type %s and 0 children\n", typeid( __Data ).name() );
		//printf( "child exists: %u\n", child_ != false );
	}
};

template<class __Data, class... __RestChildData>
static MultitypedNode<__Data, Container<__RestChildData...> > createMultitypedNode( const __Data & data, __RestChildData&&... rest_child_data )
{
	return MultitypedNode<__Data, Container<__RestChildData...> >( data, rest_child_data... );
}

template<class __MultitypedNodeType, unsigned int i>
struct child_of
{
	typedef typename child_of<__MultitypedNodeType, i - 1>::type type;
};

template<class __MultitypedNodeType>
struct child_of<__MultitypedNodeType, 0>
{
	typedef typename __MultitypedNodeType::_ChildMultitypedNode type;
};

template<class... __Types>
class MultitypedLinkedList
{
public:
	typedef Container<__Types...> _Container;
	typedef typename ContainerTypes<_Container>::_Rest _RestContainer;
	typedef typename ContainerTypes<_Container>::_Front _Front;
	typedef MultitypedNode<_Front, _RestContainer> _MultitypedNode;
	
	_MultitypedNode * root_;
	
	MultitypedLinkedList( __Types&&... data ) : root_( new _MultitypedNode( data... ) )
	{
		//
	}
	
	/*void print()
	{
		print( root_ );
	}
	
	template<class __MultitypedNodeType>
	void printMultitypedNode( __MultitypedNodeType * node )
	{
		if( node ) std::cout << node->data_ << std::endl;
	}
	
	template<class __MultitypedNodeType>
	typename std::enable_if<__MultitypedNodeType::has_child_, void>::type
	print( __MultitypedNodeType * node )
	{
		printMultitypedNode( node );
		print( node->child_ );
	}
	
	template<class __MultitypedNodeType>
	typename std::enable_if<!__MultitypedNodeType::has_child_, void>::type
	print( __MultitypedNodeType * node )
	{
		printMultitypedNode( node );
	}*/
};

template<class... __Types>
static MultitypedLinkedList<__Types...> createMultitypedLinkedList( __Types&&... data )
{
	return MultitypedLinkedList<__Types...>( data... );
}

#endif // BASE_LIBS_BASE_LIBS_MULTITYPED_LINKED_LIST_H_
