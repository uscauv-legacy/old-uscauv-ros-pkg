/***************************************************************************
 *  src/bee_stem3_driver.cpp
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

#include <seabee3_driver/bee_stem3_driver.h>

BeeStem3Driver::BeeStem3Driver(){}

BeeStem3Driver::BeeStem3Driver( std::string const & port )
{
    connect( port );
}

void BeeStem3Driver::connect( std::string const & port, bool const & force_connect )
{
    if( flags_.port_connected_ && !force_connect ) return;

    port_ = port;
    flags_.port_connected_ = bee_stem_3_.connect( port );

    dropper1_ready_ = true;
    dropper2_ready_ = true;
    shooter1_ready_ = true;
    shooter2_ready_ = true;
}

void BeeStem3Driver::reconnect( std::string const & port )
{
    connect( port, true );
}

void BeeStem3Driver::reconnect()
{
    reconnect( port_ );
}

bool const & BeeStem3Driver::connected() const
{
    return flags_.port_connected_;
}

// ######################################################################
BeeStem3Driver::~BeeStem3Driver()
{
}

// ######################################################################
void BeeStem3Driver::initPose()
{
    if( !flags_.port_connected_ ) return;

    bee_stem_3_.setPID( 0, HEADING_K, HEADING_P, HEADING_I, HEADING_D );
    bee_stem_3_.setPID( 1, DEPTH_K, DEPTH_P, DEPTH_I, DEPTH_D );

    int accelX, accelY, accelZ;
    int compassHeading, compassPitch, compassRoll;
    int internalPressure, externalPressure;
    int headingK, headingP, headingD, headingI, headingOutput;
    int depthK, depthP, depthD, depthI, depthOutput;
    int desiredHeading, desiredDepth, desiredSpeed;
    char killSwitch;

    bool successful = bee_stem_3_.getSensors( accelX, accelY, accelZ, compassHeading, compassPitch, compassRoll, internalPressure, externalPressure, desiredHeading, desiredDepth, desiredSpeed, headingK,
            headingP, headingD, headingI, headingOutput, depthK, depthP, depthD, depthI, depthOutput, killSwitch );

    if ( successful ) std::cout << "Initial communication with microcontroller completed." << std::endl;
    else std::cerr << "Initial communication with microcontroller failed." << std::endl;

    flags_.position_initialized_ = successful;
}

void BeeStem3Driver::readPressure( int & intl_pressure, int & extl_pressure )
{
    if( !flags_.port_connected_ ) return;

    int accelX, accelY, accelZ, compassHeading, compassPitch, compassRoll, desiredHeading, desiredDepth, desiredSpeed, headingK, headingP, headingD, headingI, headingOutput, depthK, depthP, depthD,
            depthI, depthOutput;
    char killSwitch;

    bee_stem_3_.getSensors( accelX, accelY, accelZ, compassHeading, compassPitch, compassRoll, intl_pressure, extl_pressure, desiredHeading, desiredDepth, desiredSpeed, headingK, headingP, headingD,
            headingI, headingOutput, depthK, depthP, depthD, depthI, depthOutput, killSwitch );

}

void BeeStem3Driver::readKillSwitch( int8_t & kill_switch )
{
    if( !flags_.port_connected_ ) return;

    int accelX, accelY, accelZ, compassHeading, compassPitch, compassRoll, desiredHeading, desiredDepth, desiredSpeed, headingK, headingP, headingD, headingI, headingOutput, depthK, depthP, depthD,
            depthI, depthOutput, intlPressure, extPressure;
    char ks;

    bee_stem_3_.getSensors( accelX, accelY, accelZ, compassHeading, compassPitch, compassRoll, intlPressure, extPressure, desiredHeading, desiredDepth, desiredSpeed, headingK, headingP, headingD,
            headingI, headingOutput, depthK, depthP, depthD, depthI, depthOutput, ks );

    kill_switch = ks;
}

bool const & BeeStem3Driver::getDeviceStatus( int const & device_id ) const
{
    static const bool default_result = false;
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
    return default_result;
}

void BeeStem3Driver::fireDevice( int device_id )
{
    if( !flags_.port_connected_ ) return;

    switch ( device_id )
    {
    case FiringDeviceIDs::shooter1:
        std::cout << "Firing torpedo! " << shooter1_params_.trigger_time_ << std::endl;
        bee_stem_3_.setThruster( MotorControllerIDs::SHOOTER, shooter1_params_.trigger_value_ );
        usleep( shooter1_params_.trigger_time_ * 1000 );
        bee_stem_3_.setThruster( MotorControllerIDs::SHOOTER, 0 );
        shooter1_ready_= false;
        break;
    case FiringDeviceIDs::shooter2:
        std::cout << "Firing torpedo! " << shooter2_params_.trigger_time_ << std::endl;
        bee_stem_3_.setThruster( MotorControllerIDs::SHOOTER, shooter2_params_.trigger_value_ );
        usleep( shooter2_params_.trigger_time_ * 1000 );
        bee_stem_3_.setThruster( MotorControllerIDs::SHOOTER, 0 );
        shooter2_ready_= false;
        break;
    case FiringDeviceIDs::dropper_stage1:
        std::cout << "Dropping first marker!" << std::endl;
        bee_stem_3_.setThruster( MotorControllerIDs::DROPPER_STAGE1, dropper1_params_.trigger_value_ );
        usleep( dropper1_params_.trigger_time_ * 1000 );
        bee_stem_3_.setThruster( MotorControllerIDs::DROPPER_STAGE1, 0 );
        dropper1_ready_ = false;
        break;
    case FiringDeviceIDs::dropper_stage2:
        std::cout << "Dropping second marker!" << std::endl;
        bee_stem_3_.setThruster( MotorControllerIDs::DROPPER_STAGE2, dropper2_params_.trigger_value_ );
        usleep( dropper2_params_.trigger_time_ * 1000 );
        bee_stem_3_.setThruster( MotorControllerIDs::DROPPER_STAGE2, 0 );
        dropper2_ready_ = false;
        break;
    }
}

void BeeStem3Driver::setThruster( int id, int value )
{
    if( !flags_.port_connected_ ) return;

    bee_stem_3_.setThruster( id, value );
}
