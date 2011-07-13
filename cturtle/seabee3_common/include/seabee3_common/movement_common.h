/*******************************************************************************
 *
 *      movement_common
 * 
 *      Copyright (c) 2010, edward
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

#ifndef MOVEMENT_COMMON_H_
#define MOVEMENT_COMMON_H_

#include <map>
#include <vector>

namespace movement_common
{
	const static int NUM_MOTOR_CONTROLLERS = 9;
	namespace MotorControllerIDs
	{
		const int FWD_RIGHT_THRUSTER = 3;
		const int FWD_LEFT_THRUSTER = 1;
		const int DEPTH_RIGHT_THRUSTER = 4;
		const int DEPTH_LEFT_THRUSTER = 2;
		const int STRAFE_FRONT_THRUSTER = 0;
		const int STRAFE_BACK_THRUSTER = 5;
		const int SHOOTER = 6;
		const int DROPPER_STAGE1 = 7;
		const int DROPPER_STAGE2 = 8;
	};

	const static int NUM_FIRING_DEVICES = 4;
	namespace FiringDeviceIDs
	{
		const int shooter1 = 0;
		const int shooter2 = 1;
		const int dropper_stage1 = 2;
		const int dropper_stage2 = 3;
	};

	namespace Axes
	{
		const int speed = 0;
		const int strafe = 1;
		const int depth = 2;

		const int roll = 3;
		const int pitch = 4;
		const int yaw = 5;

		const int speed_rel = 6;
		const int strafe_rel = 7;
		const int depth_rel = 8;

		const int roll_rel = 9;
		const int pitch_rel = 10;
		const int yaw_rel = 11;
	};

	typedef std::map<int, int> ThrusterArrayCfg;

	// define the direction of each thruster in an array that is responsible for controlling a single axis of movement
	struct AxisArrayCfg
	{
		std::vector<double> thrusters;
		double & at( const unsigned int & i )
		{
			if ( thrusters.size() < i )
			{
				thrusters.resize( i );
			}
			return thrusters.at( i - 1 );
		}
	};

}

#endif /* MOVEMENT_COMMON_H_ */
