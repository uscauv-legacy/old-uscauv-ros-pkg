/*******************************************************************************
 *
 *      filters
 * 
 *      Copyright (c) 2011, Noah Olsman olsman@usc.edu
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

#ifndef FILTERS_H_
#define FILTERS_H_
#include <common_utils/math.h>
#include <deque>
#include <stdio.h>

namespace filters
{
	typedef unsigned int _DimType;

	template<class __DataType, _DimType __Dim__>
	class MovingAverageFilter
	{
	public:
		typedef std::deque<__DataType> _Array;
		typedef typename _Array::iterator _ArrayIterator;
		const static _DimType dim_ = __Dim__;

		_Array cache_;

		MovingAverageFilter( )
		{
			//
		}

		__DataType update( __DataType current_value )
		{
			cache_.push_back( current_value );

			// make sure the size of the queue is '__Dim__'
			if( cache_.size() > __Dim__ ) cache_.pop_front();
			__DataType avg = 0;
			for( _ArrayIterator it = cache_.begin(); it != cache_.end(); ++it )
			{
				// 'it' is a pointer, so the current element is *it
				avg += *it;
			}
			avg /= cache_.size();
			return avg;
		}
	};
}
#endif /* FILTERS_H_ */
