/*******************************************************************************
 *
 *      node
 * 
 *      Copyright (c) 2010,
 *
 *      Edward T. Kaszubski (ekaszubski@gmail.com)
 *
 *      All rights reserved.
 *
 *      Redistribution and use in source and binary forms, with or without
 *      modification, are permitted provided that the following conditions are
 *      met:
 *      
 *      * Redistributions of source code must retain the above copyright
 *        notice, this list of conditions and the following disclaimer.
 *      * Redistributions in binary form must reproduce the above
 *        copyright notice, this list of conditions and the following disclaimer
 *        in the documentation and/or other materials provided with the
 *        distribution.
 *      * Neither the name of "seabee3-ros-pkg" nor the names of its
 *        contributors may be used to endorse or promote products derived from
 *        this software without specific prior written permission.
 *      
 *      THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *      "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *      LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *      A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *      OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *      SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *      LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *      DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *      THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *      (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *      OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *******************************************************************************/

#ifndef NODE_H_
#define NODE_H_

#include <libvml/activation_function.h>

#include <stdio.h>

namespace vml
{
	template<class _InternalDataType = double>
	class Node
	{
	public:
		struct Type
		{
			const static int hidden = 0;
			const static int input = 1;
			const static int output = 2;
			const static int bias = 3;
		};

		ActivationFunction<_InternalDataType> activation_function_;

		_InternalDataType activation_state_;
		int type_;

		Node( int type = Type::hidden, ActivationFunction<_InternalDataType> activation_function = ActivationFunction<_InternalDataType> () );
		virtual void stimulate( _InternalDataType input );
	};

	template<class _InternalDataType>
	Node<_InternalDataType>::Node( int type, ActivationFunction<_InternalDataType> activation_function ) :
		activation_state_( (_InternalDataType) 0.0 )
	{
		type_ = type;
		activation_function_ = activation_function;
	}


	// virtual
	template<class _InternalDataType>
	void Node<_InternalDataType>::stimulate( _InternalDataType input )
	{
		activation_state_ = activation_function_.calculateActivationState( input );
	}

	template<class _InternalDataType = double>
	class InputNode: public Node<_InternalDataType>
	{
	public:
		InputNode() :
			Node<_InternalDataType> ( Node<>::Type::input, ActivationFunction<_InternalDataType> ( ActivationFunction<>::Type::linear, 1.0 ) )
		{
			printf( "InputNode constructor\n" );
		}
	};

	template<class _InternalDataType = double>
	class OutputNode: public Node<_InternalDataType>
	{
	public:
		OutputNode() :
			Node<_InternalDataType> ( Node<>::Type::output )
		{
			printf( "OutputNode constructor\n" );
		}
	};

	template<class _InternalDataType = double>
	class BiasNode: public Node<_InternalDataType>
	{
	public:
		// _InternalDataType bias_;

		BiasNode( _InternalDataType bias = 1.0 ) :
			Node<_InternalDataType> ( Node<>::Type::bias, ActivationFunction<_InternalDataType> ( ActivationFunction<>::Type::constant, bias ) )
		{
			printf( "BiasNode constructor\n" );
		}
	};

}
#endif /* NODE_H_ */
