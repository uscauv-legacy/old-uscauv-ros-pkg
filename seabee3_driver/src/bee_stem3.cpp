/***************************************************************************
 *  src/bee_stem3.cpp
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

#include <seabee3_driver/bee_stem3.h>

#define BS_CMD_DELAY 5000000

BeeStem3::BeeStem3()
{
    initialize();
}

// ######################################################################
BeeStem3::BeeStem3( std::string const & default_device )
{
    initialize();
    connect( default_device );
}

// ######################################################################
BeeStem3::~BeeStem3()
{
    pthread_mutex_destroy( &itsSerialLock );
}

void BeeStem3::initialize()
{
    itsPort = NULL;
}

bool BeeStem3::connect( std::string const & port )
{
    printf( "making new serial object..." );
    if( itsPort ) delete itsPort;
    itsPort = new SerialPort();
    // set a default config for our serial port:
    printf( "configuring new serial object..." );
    itsPort->configure( port.c_str(), 57600, "8N1", false, false, 1 );
    //  itsPort->setBlocking(true);

    if( itsPort->connect() )
    {
        mMotorControllerState.resize( seabee3_common::movement::NUM_MOTOR_CONTROLLERS );

        for ( size_t i = 0; i < mMotorControllerState.size(); i++ )
        {
            mMotorControllerState[i] = 0;
        }

        pthread_mutex_init( &itsSerialLock, NULL);
    }

    return itsPort->connected();
}

// ######################################################################
bool BeeStem3::getSensors( int &accelX, int &accelY, int &accelZ, int &compassHeading, int &compassPitch, int &compassRoll, int &internalPressure, int &externalPressure, int &desiredHeading,
        int &desiredDepth, int &desiredSpeed, int &headingK, int &headingP, int &headingD, int &headingI, int &headingOutput, int &depthK, int &depthP, int &depthD, int &depthI, int &depthOutput,
        char &killSwitch )//,
//                          int &thruster1, int &thruster2,int &thruster3,
//int &thruster4,int &thruster5,int &thruster6)
{
    if( itsPort && !itsPort->connected() ) return false;

    char readCmd = 0x00;
    char accel_data[3];
    unsigned char adc_data[32];
    char desired_heading[2];
    char desired_depth[2];
    char desired_speed;
    char marker_drop[2];
    char comp_accel[6];
    char comp_mag[6];
    char comp_heading[6];
    char comp_tilt[6];
    char battery[4];
    char pid[12];
    char kill_switch;
    char temp;


    //clear serial buffer
    while ( itsPort->read( &temp, 1 ) )
        ;

    // send read command to Propeller
    itsPort->write( &readCmd, 1 );
    usleep( 40000 );


    //read accelerometer
    int size = itsPort->read( &accel_data, 3 );

    if ( size <= 0 )
    {
        printf( "ERROR: Couldn't read accel_data." );
        return false;
    }

    accelX = accel_data[0];
    accelY = accel_data[1];
    accelZ = accel_data[2];


    //read adc data
    size = itsPort->read( &adc_data, 32 );

    if ( size <= 0 )
    {
        printf( "ERROR: Couldn't read adc_data." );
        return false;
    }

    internalPressure = adc_data[14];
    internalPressure += adc_data[15] << 8;

    externalPressure = adc_data[12];
    externalPressure += adc_data[13] << 8;


    /*  thruster1 = adc_data[16];
     thruster1 += adc_data[17]<<8;

     thruster2 = adc_data[18];
     thruster2 += adc_data[19]<<8;

     thruster3 = adc_data[20];
     thruster3 += adc_data[21]<<8;

     thruster4 = adc_data[22];
     thruster4 += adc_data[23]<<8;

     thruster5 = adc_data[24];
     thruster5 += adc_data[25]<<8;

     thruster6 = adc_data[26];
     thruster6 += adc_data[27]<<8;*/

    //read desired heading
    size = itsPort->read( &desired_heading, 2 );

    if ( size <= 0 )
    {
        printf( "ERROR: Couldn't read desired_heading." );
        return false;
    }

    desiredHeading = desired_heading[0];
    desiredHeading += desired_heading[1] << 8;


    //read desired depth
    size = itsPort->read( &desired_depth, 2 );

    if ( size <= 0 )
    {
        printf( "ERROR: Couldn't read desired_depth." );
        return false;
    }

    desiredDepth = ( 255 & desired_depth[0] );
    desiredDepth |= ( 255 & desired_depth[1] << 8 ) & 65280;


    //read desired speed
    size = itsPort->read( &desired_speed, 1 );

    if ( size <= 0 )
    {
        printf( "ERROR: Couldn't read desired_speed." );
        return false;
    }

    desiredSpeed = desired_speed;


    //read marker droppers
    size = itsPort->read( &marker_drop, 2 );

    if ( size <= 0 )
    {
        printf( "ERROR: Couldn't read marker_drop." );
        return false;
    }

    //read compass acceleration
    size = itsPort->read( &comp_accel, 6 );

    if ( size <= 0 )
    {
        printf( "ERROR: Couldn't read comp_accel." );
        return false;
    }

    //read compass magnetic field
    size = itsPort->read( &comp_mag, 6 );

    if ( size <= 0 )
    {
        printf( "ERROR: Couldn't read comp_mag." );
        return false;
    }

    //read compass heading
    size = itsPort->read( &comp_heading, 6 );

    if ( size <= 0 )
    {
        printf( "ERROR: Couldn't read comp_heading." );
        return false;
    }

    compassHeading = (unsigned char) comp_heading[0];
    compassHeading += (unsigned char) ( comp_heading[1] ) << 8;

    compassPitch = comp_heading[2];
    compassPitch += comp_heading[3] << 8;

    compassRoll = comp_heading[4];
    compassRoll += comp_heading[5] << 8;


    //read compass tilt
    size = itsPort->read( &comp_tilt, 6 );

    if ( size <= 0 )
    {
        printf( "ERROR: Couldn't read comp_tilt." );
        return false;
    }

    //read battery values
    size = itsPort->read( &battery, 4 );

    if ( size <= 0 )
    {
        printf( "ERROR: Couldn't read battery." );
        return false;
    }

    //read pid values
    size = itsPort->read( &pid, 12 );

    if ( size <= 0 )
    {
        printf( "ERROR: Couldn't read pid." );
        return false;
    }

    headingK = pid[0];
    headingP = pid[1];
    headingD = pid[2];
    headingI = pid[3];

    headingOutput = ( 0x00ff & pid[4] );
    headingOutput |= pid[5] << 8;

    depthK = pid[6];
    depthP = pid[7];
    depthD = pid[8];
    depthI = pid[9];
    depthOutput = ( 0x00ff & pid[10] );
    depthOutput |= pid[11] << 8;


    //read killswitch value
    size = itsPort->read( &kill_switch, 1 );

    if ( size <= 0 )
    {
        printf( "ERROR: Couldn't read kill switch." );
        return false;
    }

    killSwitch = kill_switch;


    /*  printf("INFO: desired_depth[0] = %x, desired_depth[1] = %x, depthOutput= %x",
     (0x00ff & pid[10]),
     ((0x00ff & pid[11]) << 8) & 0x0ff00,
     depthOutput); */

    return true;
}

bool BeeStem3::setPID( int pidMode, float k, float p, float i, float d )
{
    if( itsPort && !itsPort->connected() ) return false;

    printf( "INFO: pidMode: %d, k %f, p %f, i %f, %f", pidMode, k, p, i, d );

    char pidCmdK;
    char pidCmdP;
    char pidCmdI;
    char pidCmdD;
    char pidDepthEn = 0x61;
    char pidHeadingEn = 0x60;
    char en = 0x01;
    char dis = 0x00;

    int16_t kv, pv, iv, dv;
    kv = k * 100;
    pv = p * 100;
    iv = i * 100;
    dv = d * 100;

    char temp;


    //clear serial buffer
    while ( itsPort->read( &temp, 1 ) )
        ;

    switch ( pidMode )
    {
    case PID_DEPTH:
        pidCmdK = 0x20;
        pidCmdP = 0x21;
        pidCmdI = 0x22;
        pidCmdD = 0x23;
        break;
    case PID_HEADING:
        pidCmdK = 0x30;
        pidCmdP = 0x31;
        pidCmdI = 0x32;
        pidCmdD = 0x33;
        break;
    case PID_DISABLE:
        printf( "INFO: Disable PID." );
        itsPort->write( &pidDepthEn, 1 );
        itsPort->write( &dis, 1 );
        itsPort->write( &pidHeadingEn, 1 );
        itsPort->write( &dis, 1 );
        return true;
        break;
    case PID_ENABLE:
        printf( "INFO: Enable PID." );
        itsPort->write( &pidDepthEn, 1 );
        itsPort->write( &en, 1 );
        //itsPort->write(&pidHeadingEn, 1);
        //itsPort->write(&en, 1);
        return true;
        break;
    default:
        printf( "ERROR: Invalid PID mode specified." );
        return false;
    }

    // send set update K cmd upper lower
    itsPort->write( &pidCmdK, 1 );

    temp = ( kv & 0xff00 ) >> 8;
    itsPort->write( &temp, 1 );
    temp = ( kv & 0x00ff );
    itsPort->write( &temp, 1 );


    // send set update P cmd
    itsPort->write( &pidCmdP, 1 );
    temp = ( pv & 0xff00 ) >> 8;
    itsPort->write( &temp, 1 );
    temp = ( pv & 0x00ff );
    itsPort->write( &temp, 1 );


    // send set update K cmd
    itsPort->write( &pidCmdI, 1 );
    temp = ( iv & 0xff00 ) >> 8;
    itsPort->write( &temp, 1 );
    temp = ( iv & 0x00ff );
    itsPort->write( &temp, 1 );


    // send set update K cmd
    itsPort->write( &pidCmdD, 1 );
    temp = ( dv & 0xff00 ) >> 8;
    itsPort->write( &temp, 1 );
    temp = ( dv & 0x00ff );
    itsPort->write( &temp, 1 );

    return true;
}

void BeeStem3::setThruster( int num, int val )
{
    if( itsPort && !itsPort->connected() ) return;
    /*! FIXED: If the kill switch is unplugged, this optimization
     *         will prevent a subset of the motors from
     *         re-starting once the kill swith is re-connected.
     *         A motor will fall into this subset if and only if
     *         its motor value is the same both before and after
     *         the kill switch is unplugged. A motor from this
     *         subset can have its behavior returned to normal if
     *         its motor value is changed after the kill switch
     *         has been re-connected.
     *
     *         This fix prevents any motors from falling into the
     *         subset mentioned above.
     *  ---------------------------------------------------------
     *  //don't bother setting it if it's already at this value
     *  if ( val == mMotorControllerState[num] )
     *  {
     *      return;
     *  }
     *  ---------------------------------------------------------
     */

    mMotorControllerState[num] = val; //save the new state
    printf( "Set thruster [%d]:%d\n", num, val );
    char thrusterCmd = 0xff;
    //char temp;

    //clear serial buffer
    //  while(itsPort->read(&temp,1));

    // send set thruster command to Propeller
    itsPort->write( &thrusterCmd, 1 );


    //  while(itsPort->read(&temp, 1) != 0)
    // std::cout << temp;
    // std::cout << std::endl;

    // send set thruster command to Propeller
    itsPort->write( &num, 1 );
    //while(itsPort->read(&temp, 1) != 0)
    //  printf("%c", temp);
    //std::cout << std::endl;

    // send set thruster command to Propeller
    itsPort->write( &val, 1 );
}

/*bool BeeStem3::setDesiredValues(int16_t heading, int16_t depth, char speed,
 char markerDropper)
 {
 char setDesiredHeadingCmd = 0x0b;
 char setDesiredDepthCmd = 0x0c;
 char temp;

 //clear serial buffer
 while(itsPort->read(&temp,1));

 // send set desired values command to Propeller
 itsPort->write(&setDesiredHeadingCmd, 1);

 char headingUpper = ((0x00ff00 & heading) >> 8) & 0x00ff;
 char headingLower = (heading & 0x00ff);

 char depthUpper = ((0x00ff00 &depth) >> 8) & 0x00ff;
 char depthLower = (depth & 0x00ff);

 printf("INFO: Writing Heading Upper %x", headingUpper);
 itsPort->write(&headingUpper,1);
 printf("INFO: Writing Heading Lower %x", headingLower);
 itsPort->write(&headingLower,1);

 itsPort->write(&setDesiredDepthCmd, 1);

 itsPort->write(&depthUpper,1);
 itsPort->write(&depthLower,1);

 itsPort->write(&speed,1);
 itsPort->write(&markerDropper,1);
 itsPort->write(&markerDropper,1);

 return true;
 }*/

bool BeeStem3::setDesiredHeading( int16_t heading )
{
    if( itsPort && !itsPort->connected() ) return false;

    char setDesiredHeadingCmd = 0x0b;
    char temp;


    //clear serial buffer
    while ( itsPort->read( &temp, 1 ) )
        ;

    // send set desired values command to Propeller
    itsPort->write( &setDesiredHeadingCmd, 1 );

    char headingUpper = ( ( 0x00ff00 & heading ) >> 8 ) & 0x00ff;
    char headingLower = ( heading & 0x00ff );

    printf( "INFO: Writing Heading Upper %x", headingUpper );
    itsPort->write( &headingUpper, 1 );
    printf( "INFO: Writing Heading Lower %x", headingLower );
    itsPort->write( &headingLower, 1 );

    return true;
}

bool BeeStem3::setDesiredDepth( int16_t depth )
{
    if( itsPort && !itsPort->connected() ) return false;

    char setDesiredDepthCmd = 0x0c;
    char temp;


    //clear serial buffer
    while ( itsPort->read( &temp, 1 ) )
        ;

    // send set desired values command to Propeller
    itsPort->write( &setDesiredDepthCmd, 1 );

    char depthUpper = ( ( 0x00ff00 & depth ) >> 8 ) & 0x00ff;
    char depthLower = ( depth & 0x00ff );

    printf( "INFO: Writing Depth Upper %x", depthUpper );
    itsPort->write( &depthUpper, 1 );
    printf( "INFO: Writing Depth Lower %x", depthLower );
    itsPort->write( &depthLower, 1 );

    return true;
}

bool BeeStem3::setDesiredSpeed( char speed )
{
    if( itsPort && !itsPort->connected() ) return false;

    char setDesiredSpeedCmd = 0x0d;
    char temp;


    //clear serial buffer
    while ( itsPort->read( &temp, 1 ) )
        ;

    // send set desired values command to Propeller
    itsPort->write( &setDesiredSpeedCmd, 1 );

    printf( "INFO: Setting speed: %d\n", speed );
    itsPort->write( &speed, 1 );

    return true;
}

void BeeStem3::startCompassCalibration()
{
    if( itsPort && !itsPort->connected() ) return;

    char startCalibCmd = 0xe0;
    char temp;


    //clear serial buffer
    while ( itsPort->read( &temp, 1 ) )
        ;


    // send set desired values command to Propeller
    itsPort->write( &startCalibCmd, 1 );
}

void BeeStem3::endCompassCalibration()
{
    if( itsPort && !itsPort->connected() ) return;

    char endCalibCmd = 0xe1;
    char temp;


    //clear serial buffer
    while ( itsPort->read( &temp, 1 ) )
        ;


    // send set desired values command to Propeller
    itsPort->write( &endCalibCmd, 1 );
}
