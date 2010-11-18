/*******************************************************************************
 *
 *      vector
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

#ifndef VECTOR_H_
#define VECTOR_H_

#include <vector>
#include <sstream>
#include <stdlib.h>
#include <stdio.h>

template<class _DataType>
class Vector
{
private:
	std::vector<_DataType> data_;

public:
	Vector( uint size = 0 )
	{
		resize( size );
	}

	void resize( uint size )
	{
		//printf( "resizing vector to %d\n", size );
		if( size != data_.size() )
			data_.resize( size );
	}

	uint size()
	{
		return data_.size();
	}

	std::string toString()
	{
		std::stringstream ss;
		ss << "{ " << data_[0];

		for ( size_t i = 1; i < data_.size(); i++ )
		{
			ss << ", " << data_[i];
		}

		ss << " }";

		return ss.str();
	}

	_DataType & at( int i )
	{
		if ( i < 0 ) return data_[i + data_.size()];
		else if ( i < data_.size() ) return data_[i];

		printf( "Invalid index: %d; acceptable range is +/-%d\n", i, data_.size() - 1 );

		return data_[0];
	}

	const _DataType & operator[]( int i ) const
	{
		return at( i );
	}

	_DataType & operator[]( int i )
	{
		return at( i );
	}

	void push_back( const _DataType & value )
	{
		data_.push_back( value );
	}

	template<class _ScalarDataType>
	Vector<_DataType> & operator*( _ScalarDataType scalar )
	{
		for( size_t i = 0; i < size(); i ++ )
		{
			data_[i] *= scalar;
		}

		return *this;
	}
};

#endif /* VECTOR_H_ */
