/*******************************************************************************
 *
 *      task
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
 *      * Neither the name of "task_fsm-RelWithDebInfo@task_fsm" nor the names of its
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

#ifndef TASK_H_
#define TASK_H_

#include <vector>
#include <common_utils/feature.h>

class Task
{
public:
	typedef unsigned int _DimType;
	typedef float _DataType;
	typedef Feature<_DataType, 3> _Feature;

	enum State
	{
		COMPLETE, INCOMPLETE_NOT_ATTEMPTED, INCOMPLETE_TIMED_OUT, INCOMPLETE_FAILED
	};


	Task * parent_;
	std::vector<Task *> subtasks_;
	_Feature location_;
	Task * active_subtask_;
	State state_;
	// other things like timers, etc go here

	Task( Task * parent = NULL,
	      _Feature location = _Feature() ) :
			parent_( parent ), location_( location ), active_subtask_( NULL ), state_( INCOMPLETE_NOT_ATTEMPTED )
	{
		//
	}

	~Task()
	{
		for ( _DimType i = 0; i < subtasks_.size(); ++i )
		{
			if ( subtasks_[i] ) delete subtasks_[i];
		}
	}

	Task * selectTask()
	{
		// return the first non-null task in subtasks_ or NULL
		if ( !active_subtask_ )
		{
			for ( _DimType i = 0; i < subtasks_.size(); ++i )
			{
				if ( subtasks_[i] ) return subtasks_[i];
			}
			return NULL;
		}

		// return the closest incomplete non-null task to active_task_ or NULL
		_DataType min_distance = std::numeric_limits < _DataType > ::max();
		Task * new_task = NULL;

		for ( _DimType i = 0; i < subtasks_.size(); ++i )
		{
			// skip completed tasks
			if ( subtasks_[i] && subtasks_[i]->state_ == COMPLETE ) continue;

			// calculate the distance to the current subtask
			const _DataType distance = active_subtask_->distance( subtasks_[i] );
			if ( distance < min_distance )
			{
				min_distance = distance;
				new_task = subtasks_[i];
			}
		}
		return new_task;
	}

	// recursively finds the next closest task, executes it, and returns the completion state of that task
	// at most one task will be executed each time this function is called
	State selectAndExecute()
	{
		active_subtask_ = selectTask();
		if ( active_subtask_ ) return active_subtask_->selectAndExecute();
		else return execute();
	}

	_DataType distance( Task * other )
	{
		return other ? location_.distance( other->location_ ) :
		               std::numeric_limits < _DataType > ::max();
	}

	virtual State execute()
	{
		return state_;
	}
};

#endif /* TASK_H_ */
