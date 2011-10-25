/***************************************************************************
 *  include/base_libs/multityped_linked_list.h
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
