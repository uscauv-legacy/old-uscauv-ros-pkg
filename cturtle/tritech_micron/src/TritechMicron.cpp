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

#include "../include/TritechMicron.h"
#include "../include/MessageTypes.h"
#include <iostream>
#include <algorithm>
    
using namespace tritech;

// ######################################################################
TritechMicron::TritechMicron() 
{ resetMessage(); }

void TritechMicron::resetMessage()
{
  itsMsg = Message();
  itsState = WaitingForAt;
  itsRawMsg.clear();
}

// ######################################################################
void TritechMicron::processByte(char byte)
{

  if(itsState == WaitingForAt) 
    if(byte == '@')
    {
      // Tritech's datasheet refers to the first byte as '1' rather than '0', so let's push back a bogus byte here just
      // to make reading the datasheet easier.
      itsRawMsg.push_back(0);
      itsRawMsg.push_back(byte);
      itsState = ReadingHeader; 
      itsMsg = Message();
      return;
    }

  itsRawMsg.push_back(byte);

  if(itsState == ReadingHeader)
  {
    // Ignore the 'Hex Length' section
    if(itsRawMsg.size() < 7) return;

    if(itsRawMsg.size() == 7)  { itsMsg.binLength  = uint16_t(byte);      return; }
    if(itsRawMsg.size() == 8)  { itsMsg.binLength |= uint16_t(byte) << 8; return; }
    if(itsRawMsg.size() == 9)  { itsMsg.txNode = byte; return; }
    if(itsRawMsg.size() == 10) { itsMsg.rxNode = byte; return; }
    if(itsRawMsg.size() == 11) { itsMsg.count  = byte; return; }
    if(itsRawMsg.size() == 12) 
    {
      itsMsg.type = MessageType(byte);
      itsState = ReadingData;
      return; 
    }

    std::cerr << "Parsing error! " << __FILE__ << ":" << __LINE__ << std::endl;
    resetMessage();
    return;
  }

  if(itsState == ReadingData)
  {
    if(int(itsMsg.binLength - (itsRawMsg.size() - 7)) == 0)
      if(byte == 0x0A)
      {
        std::cout << "Message Finished" << std::endl;
        itsMsg.data = itsRawMsg;
        processMessage(itsMsg);
        resetMessage();
      }
      else
      {
        std::cerr << "Message finished, but no LF detected!" << std::endl;
        resetMessage();
        return;
      }
  }
}

void TritechMicron::processMessage(tritech::Message msg)
{
  if(msg.type == mtVersionData)
  {
    mtVersionDataMsg parsedMsg(msg);
    parsedMsg.print(); 
  }
  else if(msg.type == mtAlive)
  {
    mtAliveMsg parsedMsg(msg);
    parsedMsg.print(); 
  }
  else if(msg.type == mtHeadData)
  {
//    mtHeadDataMsg parsedMsg(msg);
//    parsedMsg.print(); 
  }
  else
  {
    std::cerr << "Unhandled Message Type: " << msg.type << std::endl;
  }


}

