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

    enum StateType 
    {
      WaitingForAt  = 0, //!< Waiting for an @ to appear
      ReadingHeader = 1, //!< The @ sign has been found, now we're reading the header data     
      ReadingData   = 2, //!< The header has been read, now we're just reading the data
    };

    TritechMicron();
    ~TritechMicron();

    bool connect(std::string const& devName);

    void processByte(uint8_t byte);

    void processMessage(tritech::Message msg);

    void resetMessage();


  //private:
    void serialThreadMethod();
    boost::thread itsSerialThread;
    StateType itsState;
    
    //! The current message begin read in
    std::vector<uint8_t> itsRawMsg; 

    tritech::Message itsMsg;

    //! Have we ever heard an mtAlive message from the sonar? 
    bool hasHeardMtAlive;

    bool hasHeardMtVersionData;

    SerialPort itsSerial;

    bool itsRunning;

};

#endif // TRITECHMICRON_TRITECHMICRON_H

