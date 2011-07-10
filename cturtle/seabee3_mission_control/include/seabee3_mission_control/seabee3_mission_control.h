/*******************************************************************************
 *
 *      Task
 * 
 *      Copyright (c) 2011, noah
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

#ifndef TASK_H_
#define TASK_H_
#include <iostream>
using namespace std;

class Task
{
	typedef unsigned int _DimType;

	enum State
	{
		INCOMPLETE,
		SUCCESS,
		TIMED_OUT,
		FAILED
	};

	Task * parent_;
	State state_;
	std::vector<Task *> subtasks_;
	// other things like timers, etc go here

	Task( Task * parent, ... ) : parent_( parent ), state_( INCOMPLETE )
	{
		//
	}

	~Task()
	{
		for( unsigned int i = 0; i < subtasks_; ++i )
		{
			if( subtasks_[i] ) delete subtasks_[i];
		}
	}

	Task * selectTask()
	{
		// pick a task and return it
	}

	State selectAndExecute()
	{
		Task * current_task = selectTask();
		return current_task->execute();
	}

	virtual State execute() = 0;
};

class Competition : public Task
{
public:
	Competition( Task * parent, ... ) : Task( parent, ... )
	{
		subtasks_.push_back( new Gate( this ) );
		subtasks_.push_back( new Mid( this ) );
		subtasks_.push_back( new Pinger( this ) );
	}

	State execute()
	{
		// do work here
	}
};

class Mid : public Task
{
public:
	Mid( Task * parent, ... ) : Task( parent, ... )
	{
		// add two tasks of type SomeSubTask

		subtasks_.push_back( new Buoy( this ) );
		subtasks_.push_back( new Hedge( this ) );
		subtasks_.push_back( new Shooter( this ) );
		subtasks_.push_back( new Dropper( this ) );

	}

	State execute()
	{
		// do work here
	}
};

class Gate : public Task
{
public:
	Gate( Task * parent, ... ) : Task( parent, ... )
	{

	}

	State execute()
	{
		// do work here
		if(landmark_finder() != 'GATE') {
			WPC( gate.xyz, dist, 'LOOK_AROUND' );
		} else {
			moveTo(landmark_finder().xyz);
		}

		if ( timeElapsed >= taskTime ) return TIMED_OUT;
		else if ( abs( mag( seabee.xyz - gate.xyz ) ) > threshold ) return INCOMPLETE;
		else return SUCCESS;
	}
};

class Buoy : public Task
{
public:
	Buoy( Task * parent, ... ) : Task( parent, ... )
	{
		subtask_.push_back ( new FindB1 );
		subtask_.push_back ( new FindB2 );

	}

	State execute()
	{
		// do work here
	}
};

class FindB1 : public Task
{
public:
	FindB1( Task * parent, ... ) : Task( parent, ... )
	{

	}

	State execute()
	{
		// do work here
		if ( landmark_finder() != 'B1' )
		{
			WPC( B1.xyz, dist, 'LOOK_AROUND' );
		}
		else
		{
			moveTo( landmark_finder().xyz );
		}

		if ( timeElapsed >= taskTime ) return TIMED_OUT;
		else if ( abs( mag( seabee.xyz - B1.xyz ) ) > threshold ) return INCOMPLETE;
		else return SUCCESS;
	}
};

class FindB2 : public Task
{
public:
	FindB2( Task * parent, ... ) : Task( parent, ... )
	{

	}

	State execute()
	{
		// do work here
		if ( landmark_finder() != 'B2' )
		{
			WPC( B2.xyz, dist, 'LOOK_AROUND' );
		}
		else
		{
			moveTo( landmark_finder().xyz );
		}

		if ( timeElapsed >= taskTime ) return TIMED_OUT;
		else if ( abs( mag( seabee.xyz - B2.xyz ) ) > threshold ) return INCOMPLETE;
		else return SUCCESS;
	}
};

class Hedge : public Task
{
public:
	Hedge( Task * parent, ... ) : Task( parent, ... )
	{

	}

	State execute()
	{
		// do work here
		if(landmark_finder() != 'Hedge') {
			WPC( hedge.xyz, dist, 'LOOK_AROUND' );
		} else {
			moveTo(landmark_finder().xyz);
		}

		if ( timeElapsed >= taskTime ) return TIMED_OUT;
		else if ( abs( mag( seabee.xyz - gate.xyz ) ) > threshold ) return INCOMPLETE;
		else return SUCCESS;
	}
};

class Shooter : public Task
{
public:
	Shooter( Task * parent, ... ) : Task( parent, ... )
	{
		subtask_.push_back ( new FindT1 );
		subtask_.push_back ( new FindT2 );

	}

	State execute()
	{
		// do work here
	}
};

class FindT1 : public Task
{
public:
	FindT1( Task * parent, ... ) : Task( parent, ... )
	{

	}

	State execute()
	{
		// do work here
		if ( landmark_finder() != 'T1' )
		{
			WPC( T1.xyz, dist, 'LOOK_AROUND' );
		}
		else
		{
			moveTo( landmark_finder().xyz );
		}

		if ( timeElapsed >= taskTime ) return TIMED_OUT;
		else if ( abs( mag( seabee.xyz - T1.xyz ) ) > threshold ) return INCOMPLETE;
		else {
			face(T1);
			if(shoot(T1)) return SUCCESS;
			else return FAILED;
		}
	}
};

class FindT2 : public Task
{
public:
	FindT2( Task * parent, ... ) : Task( parent, ... )
	{

	}

	State execute()
	{
		// do work here
		if ( landmark_finder() != 'T2' )
		{
			WPC( T2.xyz, dist, 'LOOK_AROUND' );
		}
		else
		{
			moveTo( landmark_finder().xyz );
		}

		if ( timeElapsed >= taskTime ) return TIMED_OUT;
		else if ( abs( mag( seabee.xyz - T2.xyz ) ) > threshold ) return INCOMPLETE;
		else {
			face(T2);
			if(shoot(T2)) return SUCCESS;
			else return FAILED;
		}
	}
};

class Dropper : public Task
{
public:
	Shooter( Task * parent, ... ) : Task( parent, ... )
	{
		subtask_.push_back ( new FindD1 );
		subtask_.push_back ( new FindD2 );

	}

	State execute()
	{
		// do work here
	}
};

class FindD1 : public Task
{
public:
	FindD1( Task * parent, ... ) : Task( parent, ... )
	{

	}

	State execute()
	{
		// do work here
		if ( landmark_finder() != 'D1' )
		{
			WPC( D1.xyz, dist, 'LOOK_AROUND' );
		}
		else
		{
			moveTo( landmark_finder().xyz );
		}

		if ( timeElapsed >= taskTime ) return TIMED_OUT;
		else if ( abs( mag( seabee.xyz - D1.xyz ) ) > threshold ) return INCOMPLETE;
		else {
			center(D1);
			if(drop(D1)) return SUCCESS;
			else return FAILED;
		}
	}
};

class FindD2 : public Task
{
public:
	FindD2( Task * parent, ... ) : Task( parent, ... )
	{

	}

	State execute()
	{
		// do work here
		if ( landmark_finder() != 'D2' )
		{
			WPC( D2.xyz, dist, 'LOOK_AROUND' );
		}
		else
		{
			moveTo( landmark_finder().xyz );
		}

		if ( timeElapsed >= taskTime ) return TIMED_OUT;
		else if ( abs( mag( seabee.xyz - D2.xyz ) ) > threshold ) return INCOMPLETE;
		else {
			center(D2);
			if(drop(D2)) return SUCCESS;
			else return FAILED;
		}
	}
};

class Pinger : public Task
{
public:
	Pinger( Task * parent, ... ) : Task( parent, ... )
	{

	}

	State execute()
	{
		// do work here

	}
};

#endif /* TASK_H_ */
