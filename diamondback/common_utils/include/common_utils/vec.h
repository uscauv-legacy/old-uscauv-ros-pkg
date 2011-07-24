/*******************************************************************************
 *
 *      vec
 * 
 *      Copyright (c) 2011, Edward T. Kaszubski ( ekaszubski@gmail.com )
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
 *      * Neither the name of "common_utils-RelWithDebInfo@common_utils" nor the names of its
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

#ifndef VEC_H_
#define VEC_H_

#include <array>

typedef unsigned int _DimType;

template<class __DataType, _DimType __Dim__ = 1>
class VecBase
{
public:

	typedef __DataType _DataType;
	typedef std::array<__DataType, __Dim__> _ArrayType;
	const static _DimType _Dim_ = __Dim__;

	_ArrayType data_;

	VecBase( const _ArrayType data ) :
	data_( data )
	{
		//
	}

	VecBase()
	{
		//
	}
};

template<class __DataType>
class VecBase<__DataType, 1>
{
public:
	typedef __DataType _DataType;
	const static _DimType _Dim_ = 1;

	__DataType data_;

	VecBase( const __DataType & data = __DataType( 0 ) ) :
			data_( data )
	{
		//
	}
};

template<class __DataType, _DimType __Dim__ = 1>
class Vec: public VecBase<__DataType, __Dim__> {
public:
typedef VecBase<__DataType, __Dim__> _VecBase;
typedef std::array<__DataType, __Dim__> _ArrayType;
typedef __DataType _DataType;

Vec( const _ArrayType data ) : _VecBase( data )
{
	//
}

Vec() : _VecBase()
{
	//
}
};

template<class __DataType>
class Vec<__DataType, 1> : public VecBase<__DataType, 1>
{
public:
	typedef VecBase<__DataType, 1> _VecBase;
	typedef __DataType _DataType;

	Vec( const __DataType & data = __DataType( 0 ) ) :
			_VecBase( data )
	{
		//
	}
};

template<class __DataType>
class Vec<__DataType, 3> : public VecBase<__DataType, 3>
{
public:
	typedef VecBase<__DataType, 3> _VecBase;
	typedef std::array<__DataType, _VecBase::_Dim_> _ArrayType;
	typedef __DataType _DataType;

	__DataType * i_;
	__DataType * j_;
	__DataType * k_;

	Vec( const _ArrayType data ) :
			_VecBase( data ), i_( &this->data_[0] ), j_( &this->data_[1] ), k_( &this->data_[2] )
	{
		//
	}

	Vec( const __DataType & i,
	     const __DataType & j,
	     const __DataType & k ) :
			_VecBase( { i, j, k } ), i_( &this->data_[0] ), j_( &this->data_[1] ), k_( &this->data_[2] )
	{
		//
	}

	Vec() :
			_VecBase(), i_( &this->data_[0] ), j_( &this->data_[1] ), k_( &this->data_[2] )
	{

	}
};

#endif /* VEC_H_ */
