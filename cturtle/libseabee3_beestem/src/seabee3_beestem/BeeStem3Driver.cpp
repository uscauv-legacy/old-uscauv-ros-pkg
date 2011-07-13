/*******************************************************************************
 *
 *      BeeStem3Driver
 * 
 *      Copyright (c) 2010, Edward T. Kaszubski (ekaszubski@gmail.com), Rand Voorhies, Michael Montalbo
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
 *      * Neither the name of the USC Underwater Robotics Team nor the names of its
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

#include <seabee3_beestem/BeeStem3Driver.h>

#include <iostream>
#include <stdlib.h>
#include <sstream>

BeeStem3Driver::BeeStem3Driver( std::string port )
{
	port_ = port;
	bee_stem_3_ = new BeeStem3( port_ );
	
	flags_.init_flag_ = false;

	dropper1_ready_ = true;
	dropper2_ready_ = true;
	shooter1_ready_ = true;
	shooter2_ready_ = true;
}

// ######################################################################
BeeStem3Driver::~BeeStem3Driver()
{
}

// ######################################################################
void BeeStem3Driver::initPose()
{
	flags_.init_flag_ = true;

	bee_stem_3_->setPID( 0, HEADING_K, HEADING_P, HEADING_I, HEADING_D );
	bee_stem_3_->setPID( 1, DEPTH_K, DEPTH_P, DEPTH_I, DEPTH_D );

	int accelX, accelY, accelZ;
	int compassHeading, compassPitch, compassRoll;
	int internalPressure, externalPressure;
	int headingK, headingP, headingD, headingI, headingOutput;
	int depthK, depthP, depthD, depthI, depthOutput;
	int desiredHeading, desiredDepth, desiredSpeed;
	char killSwitch;

	bool successful = bee_stem_3_->getSensors( accelX, accelY, accelZ, compassHeading, compassPitch, compassRoll, internalPressure, externalPressure, desiredHeading, desiredDepth, desiredSpeed, headingK,
			headingP, headingD, headingI, headingOutput, depthK, depthP, depthD, depthI, depthOutput, killSwitch );

	if ( successful ) std::cout << "Initial communication with microcontroller completed." << std::endl;
	else std::cerr << "Initial communication with microcontroller failed." << std::endl;
}

void BeeStem3Driver::readPressure( int & intl_pressure, int & extl_pressure )
{
	int accelX, accelY, accelZ, compassHeading, compassPitch, compassRoll, desiredHeading, desiredDepth, desiredSpeed, headingK, headingP, headingD, headingI, headingOutput, depthK, depthP, depthD,
			depthI, depthOutput;
	char killSwitch;
	
	bee_stem_3_->getSensors( accelX, accelY, accelZ, compassHeading, compassPitch, compassRoll, intl_pressure, extl_pressure, desiredHeading, desiredDepth, desiredSpeed, headingK, headingP, headingD,
			headingI, headingOutput, depthK, depthP, depthD, depthI, depthOutput, killSwitch );

}

void BeeStem3Driver::readKillSwitch( int8_t & kill_switch )
{
	int accelX, accelY, accelZ, compassHeading, compassPitch, compassRoll, desiredHeading, desiredDepth, desiredSpeed, headingK, headingP, headingD, headingI, headingOutput, depthK, depthP, depthD,
			depthI, depthOutput, intlPressure, extPressure;
	char ks;

	bee_stem_3_->getSensors( accelX, accelY, accelZ, compassHeading, compassPitch, compassRoll, intlPressure, extPressure, desiredHeading, desiredDepth, desiredSpeed, headingK, headingP, headingD,
			headingI, headingOutput, depthK, depthP, depthD, depthI, depthOutput, ks );

	kill_switch = ks;
}

bool & BeeStem3Driver::getDeviceStatus( int device_id )
{
	static bool result = false;
	switch ( device_id )
	{
	case FiringDeviceIDs::dropper_stage1:
		return dropper1_ready_;
	case FiringDeviceIDs::dropper_stage2:
		return dropper2_ready_;
	case FiringDeviceIDs::shooter1:
		return shooter1_ready_;
	case FiringDeviceIDs::shooter2:
		return shooter2_ready_;
	}
	return result;
}

void BeeStem3Driver::fireDevice( int device_id )
{
	switch ( device_id )
	{
	case FiringDeviceIDs::shooter1:
		std::cout << "Firing torpedo! " << shooter1_params_.trigger_time_ << std::endl;
		bee_stem_3_->setThruster( MotorControllerIDs::SHOOTER, shooter1_params_.trigger_value_ );
		usleep( shooter1_params_.trigger_time_ * 1000 );
		bee_stem_3_->setThruster( MotorControllerIDs::SHOOTER, 0 );
		shooter1_ready_= false;
		break;
	case FiringDeviceIDs::shooter2:
		std::cout << "Firing torpedo! " << shooter2_params_.trigger_time_ << std::endl;
		bee_stem_3_->setThruster( MotorControllerIDs::SHOOTER, shooter2_params_.trigger_value_ );
		usleep( shooter2_params_.trigger_time_ * 1000 );
		bee_stem_3_->setThruster( MotorControllerIDs::SHOOTER, 0 );
		shooter2_ready_= false;
		break;
	case FiringDeviceIDs::dropper_stage1:
		std::cout << "Dropping first marker!" << std::endl;
		bee_stem_3_->setThruster( MotorControllerIDs::DROPPER_STAGE1, dropper1_params_.trigger_value_ );
		usleep( dropper1_params_.trigger_time_ * 1000 );
		bee_stem_3_->setThruster( MotorControllerIDs::DROPPER_STAGE1, 0 );
		dropper1_ready_ = false;
		break;
	case FiringDeviceIDs::dropper_stage2:
		std::cout << "Dropping second marker!" << std::endl;
		bee_stem_3_->setThruster( MotorControllerIDs::DROPPER_STAGE2, dropper2_params_.trigger_value_ );
		usleep( dropper2_params_.trigger_time_ * 1000 );
		bee_stem_3_->setThruster( MotorControllerIDs::DROPPER_STAGE2, 0 );
		dropper2_ready_ = false;
		break;
	}
}

void BeeStem3Driver::setThruster( int id, int value )
{
	bee_stem_3_->setThruster( id, value );
}
