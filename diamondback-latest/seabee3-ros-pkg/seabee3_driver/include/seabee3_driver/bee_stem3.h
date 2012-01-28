/***************************************************************************
 *  include/seabee3_driver/bee_stem3.h
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

#ifndef SEABEE3DRIVER_BEESTEM3_H_
#define SEABEE3DRIVER_BEESTEM3_H_

#include <seabee3_common/movement.h>
#include <quickdev/serial.h>
//#include <common_utils/types.h>      // for byte
#include <string>
#include <vector>

#define PID_DEPTH       0
#define PID_HEADING     1
#define PID_ENABLE      3
#define PID_DISABLE     2

#define DESIRED_DEPTH   0
#define DESIRED_HEADING 1

class BeeStem3
{
public:

    //! Default constructor; see ModelComponent.H
    BeeStem3( std::string default_device );

    //! Destructor
    ~BeeStem3();

    bool getSensors( int &accelX, int &accelY, int &accelZ, int &compassHeading, int &compassPitch, int &compassRoll, int &internalPressure, int &externalPressure, int &desiredHeading,
            int &desiredDepth, int &desiredSpeed, int &headingK, int &headingP, int &headingD, int &headingI, int &headingOutput, int &depthK, int &depthP, int &depthD, int &depthI, int &depthOutput,
            char &killSwitch );

    bool setPID( int pidMode, float k, float p, float i, float d );

    // bool setDesiredValues(int16_t heading, int16_t depth, char speed, char markerDropper);
    bool setDesiredHeading( int16_t heading );
    bool setDesiredDepth( int16_t depth );
    bool setDesiredSpeed( char speed );
    void setThruster( int num, int val );
    void startCompassCalibration();
    void endCompassCalibration();

    std::vector<int> mMotorControllerState;

protected:
    SerialPort * itsPort; //!< Serial port to use
    pthread_mutex_t itsSerialLock;
};

#endif // SEABEE3DRIVER_BEESTEM3_H_
