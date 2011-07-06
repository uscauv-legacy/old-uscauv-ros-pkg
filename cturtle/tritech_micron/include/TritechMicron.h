// ######################################################################
//
//      TritechMicron - A protocol parser for Tritech Micron sonars.
//      Copyright (C) 2011  Randolph Voorhies
//
//  This program is free software: you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation, either version 3 of the License, or
//  (at your option) any later version.
//
//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License
//  along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
// ######################################################################

#ifndef TRITECHMICRON_TRITECHMICRON_H
#define TRITECHMICRON_TRITECHMICRON_H

#include "Constants.h"
#include "MessageTypes.h"
#include <vector>
#include <stdint.h>
#include "Serial.h"
//#include <thread>
#include <boost/thread.hpp>

class TritechMicron
{
  public:
    TritechMicron(bool debugMode = true);

    ~TritechMicron();

    //! Connect to a Sonar device
    bool connect(std::string const& devName);

    void resetMessage();

    //! Get access to the scan lines that have been read by the sonar.
    /*! Note that this method is thread safe and will lock the internal scan
        line data structure, therefore one should call this method as
        infrequently as possible */
    std::map<float, std::vector<uint8_t>> scanLines();

    void registerScanLineCallback(
        std::function<void(float/*angle*/, float/*meters per bin*/, std::vector<uint8_t>/*scanline*/)>);

  private:

    //! Process a single incoming byte (add it onto itsRawMsg, etc.)
    void processByte(uint8_t byte);

    //! Process an incoming message
    void processMessage(tritech::Message msg);

    //! The method being run by itsSerialThread
    void serialThreadMethod();
    
    //! A thread that just spins and reads data from the serial port
    boost::thread itsSerialThread;

    //! Current state of the incoming protocol FSM
    enum StateType 
    {
      WaitingForAt  = 0, //!< Waiting for an @ to appear
      ReadingHeader = 1, //!< The @ sign has been found, now we're reading the header data     
      ReadingData   = 2, //!< The header has been read, now we're just reading the data
    };

    //! The current state of the FSM controlling the parsing of the incoming protocol
    StateType itsState;
    
    //! The current message buffer begin read in
    std::vector<uint8_t> itsRawMsg; 

    //! The current incoming message begin constructed from itsRawMsg
    tritech::Message itsMsg;

    //! Have we ever heard an mtAlive message from the sonar? 
    bool hasHeardMtAlive;

    //! Have we ever heard an mtVersionData from the sonar?
    bool hasHeardMtVersionData;

    //! The actual serial port
    SerialPort itsSerial;

    //! Are we currently running?
    bool itsRunning;

    //! Should we be printing debugging information to the console? Warning, this is very noisy and slow.
    bool itsDebugMode;

    std::function<void(float/*angle*/, float/*meters per bin*/, std::vector<uint8_t>/*scanline*/)> itsScanLineCallback;
};

#endif // TRITECHMICRON_TRITECHMICRON_H

