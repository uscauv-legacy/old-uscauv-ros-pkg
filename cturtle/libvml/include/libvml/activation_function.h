/*******************************************************************************
 *
 *      activation_function
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

#ifndef ACTIVATION_FUNCTION_H_
#define ACTIVATION_FUNCTION_H_

#include <math.h>
#include <stdlib.h>

namespace vml
{

	template<class _InternalDataType = double>
	class ActivationFunction
	{
	public:
		struct Type
		{
			const static uint constant = 0;
			const static uint piecewise = 1;
			const static uint linear = 2;
			const static uint sigmoid = 3;
		};

	private:
		uint type_;
		_InternalDataType coefficient_;

	public:
		ActivationFunction( uint type = Type::sigmoid, _InternalDataType coefficient = (_InternalDataType) 0.0 );
		_InternalDataType calculateActivationState( _InternalDataType input );
	};

	template<class _InternalDataType>
	ActivationFunction<_InternalDataType>::ActivationFunction( uint type, _InternalDataType coefficient )
	{
		type_ = type;
		coefficient_ = coefficient;
	}

	template<class _InternalDataType>
	_InternalDataType ActivationFunction<_InternalDataType>::calculateActivationState( _InternalDataType input )
	{
		switch ( type_ )
		{
		case Type::constant:
			return coefficient_;
		case Type::piecewise:
			return round( input );
		case Type::linear:
			return coefficient_ * input;
		case Type::sigmoid:
			return 1.0 / ( 1.0 + pow( M_E, -input ) );
		}

		return 0.0;
	}
}

#endif /* ACTIVATION_FUNCTION_H_ */
