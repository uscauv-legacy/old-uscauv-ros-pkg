/***************************************************************************
 *  include/pipe_finder/histogram.h
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
 *  * Neither the name of pipe_finder nor the names of its
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

#ifndef PIPEFINDER_PIPEFINDER_HISTOGRAM_H_
#define PIPEFINDER_PIPEFINDER_HISTOGRAM_H_

#include <vector>
#include <limits>

template<class __Data>
struct Histogram
{
	typedef std::vector<__Data> _DataVec;
	typedef typename _DataVec::iterator iterator;

	_DataVec bins_;
	__Data min_;
	__Data max_;
	unsigned int min_index_;
	unsigned int max_index_;
	bool modified_;

	Histogram( unsigned int size = 0 ) : min_ ( std::numeric_limits<__Data>::min() ), max_( std::numeric_limits<__Data>::max() ), min_index_( 0 ), max_index_( 0 ), modified_( false )
	{
		bins_.reserve( size );
	}

	void push_back( __Data bin )
	{
		bins_.push_back( bin );
		if( bin < min_ )
		{
			min_ = bin;
			min_index_ = size() - 1;
		}

		if( bin > max_ )
		{
			max_ = bin;
			max_index_ = size() - 1;
		}
	}

	__Data & operator[]( unsigned int i )
	{
		return bins_[i];

		modified_ = true;
	}

	const __Data & operator[]( unsigned int i ) const
	{
		return bins_[i];
	}

	iterator begin()
	{
		return bins_.begin();
	}

	iterator end()
	{
		return bins_.end();
	}

	size_t size() const
	{
		return bins_.size();
	}

	void updateStatistics( bool force = false )
	{
		if( !modified_ && !force ) return;

		unsigned int i = 0;
		for( auto bin = bins_.begin(); bin != bins_.end(); ++bin, ++i )
		{
			if( *bin > max_ )
			{
				max_ = *bin;
				max_index_ = i;
			}

			if( *bin < min_ )
			{
				min_ = *bin;
				min_index_ = i;
			}
		}
	}
};

#endif // PIPEFINDER_PIPEFINDER_HISTOGRAM_H_
