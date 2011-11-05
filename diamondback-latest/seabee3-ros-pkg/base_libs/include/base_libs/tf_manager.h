/***************************************************************************
 *  include/base_libs/tf_manager.h
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

#ifndef BASE_LIBS_BASE_LIBS_TF_MANAGER_H_
#define BASE_LIBS_BASE_LIBS_TF_MANAGER_H_

#include <base_libs/console.h>
#include <string>
#include <map>
#include <tf/transform_datatypes.h>

namespace base_libs
{

/*! Utility class to store and update (but not publish) a list of transforms */

class TfManager
{
public:
	typedef std::string _TfFrameId;
	typedef tf::StampedTransform _Transform;
	typedef std::map<_TfFrameId, _Transform> _TransformMap;

private:
	_TransformMap transforms_;

public:
	TfManager()
	{
		//
	}

	template<class... __Rest>
	void updateTransforms( const _Transform & transform, __Rest&&... rest )
	{
		if( transform.child_frame_id_.size() == 0 || transform.frame_id_.size() == 0 )
			PRINT_WARN( "Cannot update transform with empty source frame or target frame id:\n[ %s -> %s ] : %f", transform.frame_id_.c_str(), transform.child_frame_id_.c_str(), transform.stamp_.toSec() );
		else
			transforms_[transform.child_frame_id_] = transform;

		updateTransforms( rest... );
	}
	void updateTransforms(){}

	template<class... __Rest>
	void removeTransforms( const _TfFrameId & frame_id, __Rest&&... rest )
	{
		warnIfFrameDNE( frame_id );

		transforms_.erase( frame_id );

		removeTransforms( rest... );
	}
	void removeTransforms(){}

	const _TransformMap & getTransforms() const
	{
		return transforms_;
	}

	bool exists( const _TfFrameId & frame_id ) const
	{
		return transforms_.count( frame_id );
	}

	const _Transform & operator[]( const _TfFrameId & frame_id ) const
	{
		if( !warnIfFrameDNE( frame_id ) ) return getEmptyTransform();
		return transforms_.find( frame_id )->second;
	}

	bool warnIfFrameDNE( const _TfFrameId & frame_id ) const
	{
		if( exists( frame_id ) ) return true;

		PRINT_WARN( "Cannot get frame [ %s ]; it is not managed by this TfManager", frame_id.c_str() );
		return false;
	}

	const _Transform & getEmptyTransform() const
	{
		const static _Transform empty_transform = _Transform();
		return empty_transform;
	}
};

}

#endif // BASE_LIBS_BASE_LIBS_TF_MANAGER_H_
