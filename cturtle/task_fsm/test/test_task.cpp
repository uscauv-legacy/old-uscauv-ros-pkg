/*******************************************************************************
 *
 *      test_task
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

#include <task_fsm/task.h>

class SomeSubTask: public Task
{
public:
	SomeSubTask( Task * parent = NULL ) :
			Task( parent )
	{
		//
	}

	State execute()
	{
		return COMPLETE;
	}
};

class SomeTask: public Task
{
public:
	SomeTask( Task * parent = NULL ) :
			Task( parent )
	{
		subtasks_.push_back( new SomeSubTask( this ) );
		subtasks_.push_back( new SomeSubTask( this ) );
		subtasks_.push_back( new SomeSubTask( this ) );
	}

	State execute()
	{
		return COMPLETE;
	}
};

void example()
{
	SomeTask some_task;
	some_task->selectAndExecute();

	/* --find child tasks of some_task
	 * > some_task->slectAndExecute()
	 *   > some_task->selectTask()
	 *     > active_subtask_ is null; return first non-null subtask
	 *     < subtasks_[0] is not null; return it
	 *   < some_task->subtasks_[0]
	 *
	 *   --find child tasks of some_task->subtasks_[0]
	 *   > some_task->subtasks_[0]->selectAndExecute()
	 *     > some_task->subtasks_[0]->selectTask()
	 *       > active_subtask_ is null; return first non-null subtask
	 *       < subtasks_.size() is 0; return NULL
	 *     < NULL
	 *
	 *   --leaf reached; execute
	 *   > some_task->subtasks_[0]->execute();
	 *   < complete
	 * < complete
	 */
}

