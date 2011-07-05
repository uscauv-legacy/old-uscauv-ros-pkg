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
#include <chrono>
    
using namespace tritech;

// ######################################################################
TritechMicron::TritechMicron() 
{
  bool hasHeardMtAlive = false;
  bool hasHeardMtVersionData = false;
  resetMessage(); 
}

// ######################################################################
TritechMicron::~TritechMicron()
{
  itsRunning = false;
  if(itsSerialThread.joinable())
    itsSerialThread.join();
}

// ######################################################################
bool TritechMicron::connect(std::string const& devName)
{
  std::cout << "Connecting...";

  bool connectionSuccess = itsSerial.connect(devName, B115200); 
  if(!connectionSuccess)
  {
    std::cerr << "Could not connect to serial port!" << std::endl;
    return false;
  }
  std::cout << "Connected" << std::endl;

  sleep(1);

  auto start = std::chrono::monotonic_clock::now();
  while(!hasHeardMtVersionData)
  {
    // Send a new mtSendVersionMsg once a second
    if(std::chrono::monotonic_clock::now()-start > std::chrono::seconds(1))
    {
      itsSerial.writeVector(mtSendVersionMsg);
      start = std::chrono::monotonic_clock::now();
    }

    // Keep reading bytes from the serial port
    std::vector<uint8_t> bytes = itsSerial.read(1);
    if(bytes.size() > 0)
      for(uint8_t const& byte : bytes) processByte(byte);
  }

  itsSerial.writeVector(mtHeadCommandMsg);

  while(1)
  {
  itsSerial.writeVector(mtSendDataMsg);
  sleep(1);
  }

  return true;
}

// ######################################################################
void TritechMicron::serialThreadMethod()
{
  while(itsRunning)
  {
    std::vector<uint8_t> bytes = itsSerial.read(1);
    if(bytes.size() > 0)
      for(uint8_t const& byte : bytes)
        processByte(byte);
    else { usleep(100000); }
  }
}

// ######################################################################
void TritechMicron::resetMessage()
{
  itsMsg = Message();
  itsState = WaitingForAt;
  itsRawMsg.clear();
}

// ######################################################################
void TritechMicron::processByte(uint8_t byte)
{
  if(itsState == WaitingForAt) 
    if(byte == '@')
    {
      itsRawMsg.clear();
      // Tritech's datasheet refers to the first byte as '1' rather than '0', so let's push back a bogus byte here just
      // to make reading the datasheet easier.
      itsRawMsg.push_back(0);
      itsRawMsg.push_back(byte);
      itsState = ReadingHeader; 
      itsMsg = Message();
      return;
    }
    else
    {
      std::cout << "bogus byte: " << std::hex << int(byte) << std::endl;
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
      std::cout << "MsgType: " << int(byte) << std::endl;
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

// ######################################################################
void TritechMicron::processMessage(tritech::Message msg)
{
  if(msg.type == mtVersionData)
  {
    mtVersionDataMsg parsedMsg(msg);
    if(hasHeardMtVersionData == false)
    {
      std::cout << "Received mtVersionData Message" << std::endl;
      parsedMsg.print(); 
    }
    hasHeardMtVersionData = true;
  }
  else if(msg.type == mtAlive)
  {
    mtAliveMsg parsedMsg(msg);
    if(hasHeardMtAlive == false)
    {
      std::cout << "Received mtAlive Message" << std::endl;
      parsedMsg.print();
      std::cout << "Msg bytes: [";
      for(uint8_t b : msg.data)
        std::cout << " 0x" << std::hex << int(b);
      std::cout << " ]" << std::endl;
    }
    hasHeardMtAlive = true;
  }
  else if(msg.type == mtHeadData)
  {
    mtHeadDataMsg parsedMsg(msg);
    parsedMsg.print(); 
    std::cout << "Received mtHeadData Message" << std::endl;
  }
  else if(msg.type == mtBBUserData)
  {
    std::cout << "Received mtBBUserData Message" << std::endl;
  }
  else
  {
    std::cerr << "Unhandled Message Type: " << msg.type << std::endl;
  }
}

