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
#include <ros/ros.h>
#include <thread>
#include <functional>

class Task
{
public:
	typedef unsigned int _DimType;
	typedef float _DataType;
	typedef Feature<_DataType, 3> _Feature;// other things like timers, etc go here

	enum State
	{
		COMPLETE, ATTEMPTING, INCOMPLETE_NOT_ATTEMPTED, INCOMPLETE_TIMED_OUT, INCOMPLETE_FAILED
	};

	std::thread * m_thread_;

	Task * parent_;
	std::vector<Task *> subtasks_;
	_Feature location_;
	Task * active_subtask_;
	State state_;
	ros::Timer timer_;
	std::string name_;
	ros::NodeHandle nh_local_;
	bool run_;
	bool new_task_;

	Task( Task * parent = NULL,
	      std::string name = "",
	      _Feature location = _Feature(),
	      float timer_duration = 0 ) :
			parent_( parent ), location_( location ), active_subtask_( NULL ), state_( INCOMPLETE_NOT_ATTEMPTED ), name_( name ), nh_local_( parent->nh_local_, name ), run_( true ), new_task_( false )
	{
		timer_ = nh_local_.createTimer( ros::Duration( timer_duration ), &Task::timedOutCB, this, true );
		m_thread_ = new std::thread( &Task::run(), this );
	}

	~Task()
	{
		for ( _DimType i = 0; i < subtasks_.size(); ++i )
		{
			if ( subtasks_[i] ) delete subtasks_[i];
		}
	}

	void start()
	{
		run_ = true;
		new_task_ = true;
	}

	void stop()
	{
		run_ = false;
	}

	void run()
	{
		while( run_ && new_task_ )
		{
			new_task_ = false;
			if( selectAndExecute() ) execute_0();
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

		// return active_subtask_ or the closest incomplete non-null task to active_task_ or NULL

		if( active_subtask_->state_ == ATTEMPTING ) return active_subtask_;

		_DataType min_distance = std::numeric_limits < _DataType > ::max();
		Task * new_task = NULL;

		for ( _DimType i = 0; i < subtasks_.size(); ++i )
		{
			// skip completed tasks
			if ( !subtasks_[i] || subtasks_[i]->state_ == COMPLETE || subtasks_[i]->state_ == ATTEMPTING ) continue;

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

	// recursively locate a leaf task and run it
	// this will eventually result in a new thread being created for the leaf task to execute in
	// returns true if this tasks's execute method should be started; false otherwise
	bool selectAndExecute()
	{
		// get the closest subtask
		active_subtask_ = selectTask();

		state_ = ATTEMPTING;
		timer_.start();

		// at least one INCOMPLETE_ subtask found
		if ( active_subtask_ )
		{
			// start our timer and prepare to activate this subtask
			active_subtask_->selectAndExecute();

			// return false once a valid subtask is found and started
			return false;
			//m_thread_ = new std::thread( &Task::selectAndExecute, active_subtask_ );
		}
		// all subtasks complete
		else
		{
			// return true if we should start our task
			return true;
		}
	}

	_DataType distance( Task * other )
	{
		return other ? location_.distance( other->location_ ) :
		               std::numeric_limits < _DataType > ::max();
	}

	void timedOutCB( const ros::TimerEvent& e )
	{
		if( state_ == ATTEMPTING )
		{
			// stop our timer if we were attempting the task
			if( timer_.hasPending() ) timer_.stop();

			printf( "Timer %s timed out\n" );
			state_ = INCOMPLETE_TIMED_OUT;
		}

		for( _DimType i = 0; i < subtasks_.size(); ++i )
		{
			subtasks_[i]->timedOutCB( e );
		}

		parent_->subtaskComplete();
	}

	void subtaskComplete()
	{
		new_task_ = true;
	}

	void execute_0()
	{
		setup_0();
		state_ = execute();
		timer_.stop();
		parent_->subtaskComplete();
	}

	virtual void execute()
	{
		//
	}

	void setup_0()
	{
		setup();
	}

	virtual void setup()
	{
		//
	}

};

int main( int argc, char ** argv )
{
	Task top_level_task;
	// add subtasks, etc

	top_level_task.start();

	while( top_level_task.state_ == Task::State::ATTEMPTING )
	{
		// wait for all tasks to complete
		// optionally query top-level task for subtask info
	}

	return 0;
}

#endif /* TASK_H_ */
