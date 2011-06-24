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
      itsRawMsg.push_back(byte);
      itsState = ReadingHeader; 
      itsMsg = Message();
      return;
    }

  itsRawMsg.push_back(byte);

  if(itsState == ReadingHeader)
  {
    // Ignore the 'Hex Length' section
    if(itsRawMsg.size() < 6) return;

    if(itsRawMsg.size() == 6)  { itsMsg.binLength += uint16_t(byte);      return; }
    if(itsRawMsg.size() == 7)  { itsMsg.binLength += uint16_t(byte) << 8; return; }
    if(itsRawMsg.size() == 8)  { itsMsg.txNode = byte; return; }
    if(itsRawMsg.size() == 9)  { itsMsg.rxNode = byte; return; }
    if(itsRawMsg.size() == 10) { itsMsg.count  = byte; return; }
    if(itsRawMsg.size() == 11) 
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
    if(itsRawMsg.size() - itsMsg.count == 11)
      if(byte == 0x0A)
      {
        std::cout << "Message Finished" << std::endl;
        itsMsg.data.resize(itsMsg.count-1);
        std::copy(itsRawMsg.begin()+11, itsRawMsg.end()-1, itsMsg.data.begin());
        processMessage(itsMsg);
      }
      else
      {
        std::cerr << "Message finished, but no LF detected!" << std::endl;
        resetMessage();
        return;
      }
  }
}

void TritechMicron::processMessage(TritechMicron::Message msg)
{
  std::cout << "Got Data: ";
  for(uint8_t byte : msg.data)
    printf("0x%x ", byte);
  std::cout << std::endl;

  if(msg.type == mtVersionData)
  {
    std::cout << "Handling Version Data Message" << std::endl;
  }
}

