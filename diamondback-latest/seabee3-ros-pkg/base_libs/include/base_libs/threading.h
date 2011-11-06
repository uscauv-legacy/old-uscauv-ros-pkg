/***************************************************************************
 *  include/base_libs/threading.h
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

#ifndef BASE_LIBS_BASE_LIBS_THREADING_H_
#define BASE_LIBS_BASE_LIBS_THREADING_H_

#include <boost/thread/mutex.hpp>

namespace base_libs
{
	class Mutex
	{
	public:
		boost::mutex mutex_;
		typedef boost::unique_lock<boost::mutex> _UniqueLock;

		_UniqueLock getLock()
		{
			return _UniqueLock( mutex_, boost::try_to_lock_t() );
		}
	};

	template<class __Storage>
	class MutexedCache : public Mutex
	{
	public:
		__Storage cache_;

		_UniqueLock tryLockAndUpdate( const __Storage & cache )
		{
			auto lock = getLock();
			if( lock ) cache_ = cache;
			return lock;
		}
	};

	template<class __Message>
	class MessageCache : public MutexedCache<typename __Message::ConstPtr>{};

	template<>
	class MessageCache<void>{};
}

#endif // BASE_LIBS_BASE_LIBS_THREADING_H_
