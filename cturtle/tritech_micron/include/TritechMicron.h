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

#include "Constants.h"
#include <vector>
#include <stdint.h>

class TritechMicron
{
  public:

    enum StateType 
    {
      WaitingForAt  = 0, //!< Waiting for an @ to appear
      ReadingHeader = 1, //!< The @ sign has been found, now we're reading the header data     
      ReadingData   = 2, //!< The header has been read, now we're just reading the data
    };

    struct Message
    {
      uint16_t binLength; //!< The expected data section length of the message
      uint8_t txNode;        
      uint8_t rxNode;
      uint8_t count;
      std::vector<uint8_t> data;
      tritech::MessageType type;
    };

    TritechMicron();

    void processByte(char byte);

    void processMessage(Message msg);

    void resetMessage();

  //private:
    StateType itsState;
    std::vector<uint8_t> itsRawMsg; //!< The current message begin read in
    Message itsMsg;

};
