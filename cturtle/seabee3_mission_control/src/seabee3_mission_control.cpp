/*******************************************************************************
 *
 *      seabee3_mission_control
 *
 *      Copyright (c) 2010
 *
 *      Mike Gerow
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

#include <ros/ros.h>
#include <vector>

enum {STATE_INIT, STATE_DO_GATE, STATE_FIRST_BOUY, STATE_SECOND_BOUY, STATE_HEDGE, STATE_FIRST_BIN, STATE_SECOND_BIN, STATE_WINDOW, STATE_PINGER};

class State
{
private:
	int switch_to;
	int prev_state;

public:
	int this_state;
	State * prev_state_ptr;
	static ros::NodeHandle & nh;

	State()
	{
		this_state = -1;
		switch_to = -1;
	}
	virtual void init()
	{
		//runs once when machine is initialized
	}
	virtual void entry()
	{
		//runs once on entry
	}
	virtual void loop()
	{
		//loops main execution of the function
	}
	virtual void exit()
	{
		//runs once on exit
	}
	void switchState(int state_to_switch_to)
	{
		switch_to = state_to_switch_to;
	}
	int getLastState()
	{
		return prev_state;
	}
	int spin(int prev_state_, State * prev_state_ptr_)
	{
		prev_state_ptr_ = prev_state_ptr;
		prev_state = prev_state_;
		entry();
		while(ros::ok() && (switch_to == -1))
		{
			loop();
		}
		exit();
		return switch_to;
	}
};

//ros::NodeHandle & State::nh;

class Machine
{
private:
	std::vector<State> state_vector;

public:
	int state;
	int prev_state;
	ros::NodeHandle & nh;

	Machine(ros::NodeHandle & nodehandle, std::vector<State> & state_vector_) : nh(nodehandle)
	{
		state = STATE_INIT;
		prev_state = STATE_INIT;
		state_vector = state_vector_;
		//nh = nodehandle;
	}
	void spin()
	{
		for (unsigned int i = 0; i < state_vector.size(); i++)
		{
			state_vector[i].init();
		}
		while (ros::ok())
		{
			int new_state;
			new_state = state_vector[state].spin(prev_state, &state_vector[prev_state]);
			prev_state = state;
			state = new_state;
		}
	}
};

///---------------INIT -----------------///
class StateInit: public State
{
private:
	//anything the State might need
public:
	//stuff
	void init()
	{
		this_state = STATE_INIT;
		//code that needs to run on init
	}
	void entry()
	{
		//code that needs to run when this state is entered
	}
	void loop()
	{
		//code that needs to run every time the state loops
	}
	void exit()
	{
		//code that needs to run on exit
	}
};

///-------------DO GATE ---------------///
class StateDoGate: public State
{
private:
	//anything the state might need
public:
	//stuff
	void init()
	{
		this_state = STATE_DO_GATE;
		//code that needs to run on init
	}
	void entry()
	{
		//code that needs to run when this state is entered
	}
	void loop()
	{
		//code that needs to run every time the state loops
	}
	void exit()
	{
		//code that needs to run on exit
	}
};

///----------------- FIRST BOUY --------------------/////
class StateFirstBouy: public State
{
private:
	//anything the state might need
public:
	//stuff
	void init()
	{
		this_state = STATE_FIRST_BOUY;
		//code that needs to run on init
	}
	void entry()
	{
		//code that needs to run when this state is entered
	}
	void loop()
	{
		//code that needs to run every time the state loops
	}
	void exit()
	{
		//code that needs to run on exit
	}
};

///------------------- SECOND BOUY ---------------------////
class StateSecondBouy: public State
{
private:
	//anything the state might need
public:
	//stuff
	void init()
	{
		this_state = STATE_SECOND_BOUY;
		//code that needs to run on init
	}
	void entry()
	{
		//code that needs to run when this state is entered
	}
	void loop()
	{
		//code that needs to run every time the state loops
	}
	void exit()
	{
		//code that needs to run on exit
	}
};

///--------------- HEDGE --------------------///
class StateHedge: public State
{
private:
	//anything the state might need
public:
	//stuff
	void init()
	{
		this_state = STATE_HEDGE;
		//code that needs to run on init
	}
	void entry()
	{
		//code that needs to run when this state is entered
	}
	void loop()
	{
		//code that needs to run every time the state loops
	}
	void exit()
	{
		//code that needs to run on exit
	}
};

///----------------FIRST BIN -------------------------///
class StateFirstBin: public State
{
private:
	//anything the state might need
public:
	//stuff
	void init()
	{
		this_state = STATE_FIRST_BIN;
		//code that needs to run on init
	}
	void entry()
	{
		//code that needs to run when this state is entered
	}
	void loop()
	{
		//code that needs to run every time the state loops
	}
	void exit()
	{
		//code that needs to run on exit
	}
};

///---------------- SECOND BIN ---------------///
class StateSecondBin: public State
{
private:
	//anything the state might need
public:
	//stuff
	void init()
	{
		this_state = STATE_SECOND_BIN;
		//code that needs to run on init
	}
	void entry()
	{
		//code that needs to run when this state is entered
	}
	void loop()
	{
		//code that needs to run every time the state loops
	}
	void exit()
	{
		//code that needs to run on exit
	}
};

 /// ------------------ WINDOW ----------------////
class StateWindow: public State
{
private:
	//anything the state might need
public:
	//stuff
	void init()
	{
		this_state = STATE_WINDOW;
		//code that needs to run on init
	}
	void entry()
	{
		//code that needs to run when this state is entered
	}
	void loop()
	{
		//code that needs to run every time the state loops
	}
	void exit()
	{
		//code that needs to run on exit
	}
};

/// ----------------- PINGER ---------------///
class StatePinger: public State
{
private:
	//anything the state might need
public:
	//stuff
	void init()
	{
		this_state = STATE_PINGER;
		//code that needs to run on init
	}
	void entry()
	{
		//code that needs to run when this state is entered
	}
	void loop()
	{
		//code that needs to run every time the state loops
	}
	void exit()
	{
		//code that needs to run on exit
	}
};

ros::NodeHandle nh;
ros::NodeHandle & State::nh = nh;

int main(int argc, char ** argv)
{
	ros::init(argc, argv, "seabee3_mission_control");
	ROS_INFO("Starting up...");
	//ros::NodeHandle nh;
	//ros::NodeHandle & State::nh = nh;

	std::vector<State> states;

	states.push_back(StateInit());
	states.push_back(StateDoGate());
	states.push_back(StateFirstBouy());
	states.push_back(StateSecondBouy());
	states.push_back(StateHedge());
	states.push_back(StateFirstBin());
	states.push_back(StateSecondBin());
	states.push_back(StateWindow());
	states.push_back(StatePinger());

	Machine m(nh, states);

	m.spin();

	return 0;
}
