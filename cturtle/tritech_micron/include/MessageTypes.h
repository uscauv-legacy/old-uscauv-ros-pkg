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

#ifndef TRITECHMICRON_MESSAGETYPES_H
#define TRITECHMICRON_MESSAGETYPES_H

#include "TritechMicron.h"
#include <iostream>
#include <vector>
#include <bitset>

#define TRITECH_MSG_LENGTH_CHK(length) \
	if(msg.data.size() != length)     \
	{ \
		std::cerr << __FUNCTION__ << ":" << __LINE__ << \
		" - Invalid message length (" << msg.data.size() << " != " << length << ")" << std::endl; \
		return; \
	} \
	if(msg.type != type) \
	{ \
		std::cerr << __FUNCTION__ << ":" << __LINE__ <<  \
		" - Invalid message type (" << msg.type << " != " << type << ")" << std::endl; \
		return; \
	}

#define TRITECH_MSG_EXPECT_BYTE_AT(byte, at) \
	if(msg.data.size() < at) \
	{ \
		std::cerr << __FUNCTION__ << ":" << __LINE__ <<  \
		" - Message too short" << std::endl; \
		return; \
	}  \
	if(msg.data[at] != byte) \
	{                        \
		std::cerr << __FUNCTION__ << ":" << __LINE__ <<  \
		" - Expected " << std::hex << (int)byte << std::dec << " @" << \
		at << " but got " << std::hex << (int)msg.data[at] << std::endl;\
		return; \
	}


namespace tritech
{
  struct Message
  {
    uint16_t binLength;
    uint8_t txNode;        
    uint8_t rxNode;
    uint8_t count;
    std::vector<uint8_t> data;
    tritech::MessageType type;
  };

	// ######################################################################
	static std::vector<uint8_t> mtSendVersionMsg = 
	{ 0x40, 0x30, 0x30, 0x30, 0x38, 0x08, 0x00, 0xFF, 0x02, 0x03, 0x17, 0x80, 0x02, 0x0A };

	// ######################################################################
	struct mtVersionDataMsg
	{
		static const int type = mtVersionData;

		uint8_t node;
		uint8_t softwareVersion;
		uint8_t infoBits;
		uint16_t uid;
		uint32_t programLength;
		uint16_t checksum;

		mtVersionDataMsg(Message const& msg)
		{
			TRITECH_MSG_LENGTH_CHK(26);
			TRITECH_MSG_EXPECT_BYTE_AT('@', 1);
			TRITECH_MSG_EXPECT_BYTE_AT(mtVersionData, 11);

			node = msg.data[13];

			softwareVersion = msg.data[14];

			infoBits = msg.data[15];

			uid =  uint16_t(msg.data[16] << 0);
			uid |= uint16_t(msg.data[17] << 8);

			programLength  = uint32_t(msg.data[18] << 0);
			programLength |= uint32_t(msg.data[19] << 8);
			programLength |= uint32_t(msg.data[20] << 16);
			programLength |= uint32_t(msg.data[21] << 24);

			checksum  = uint32_t(msg.data[22] << 0);
			checksum += uint32_t(msg.data[23] << 8);

			TRITECH_MSG_EXPECT_BYTE_AT(0x0A, 25);

		}

		void print()
		{
			printf("mtVersionDataMsg: node = %#x swVer = %#x infoBits = %#x uid = %#x programLength = %d checksum = %d\n",
					node,
					softwareVersion,
					infoBits,
					uid,
					programLength,
					checksum);
		}
	};

	// ######################################################################
	struct mtAliveMsg
	{
		static const int type = mtAlive;
		uint8_t txNode;
		uint32_t headTime_msec;
		int motorPos;

		bool inCentre;
		bool centered;
		bool motoring;
		bool motorOn;
		bool dir;
		bool inScan;
		bool noParams;
		bool sentCfg;

		mtAliveMsg(Message const& msg)
		{
			TRITECH_MSG_EXPECT_BYTE_AT('@', 1);
			TRITECH_MSG_LENGTH_CHK(23);
			TRITECH_MSG_EXPECT_BYTE_AT(mtAlive, 11);
			TRITECH_MSG_EXPECT_BYTE_AT(0x80, 12);

			txNode = msg.data[13];

			headTime_msec  = uint32_t(msg.data[15]) << 0;
			headTime_msec |= uint32_t(msg.data[16]) << 8;
			headTime_msec |= uint32_t(msg.data[17]) << 16;
			headTime_msec |= uint32_t(msg.data[18]) << 24;

			motorPos  = msg.data[19] << 0;
			motorPos |= msg.data[20] << 8;

			std::bitset<8> headinf = msg.data[21];
			inCentre  = headinf[0];
			centered  = headinf[1];
			motoring  = headinf[2];
			motorOn   = headinf[3];
			dir       = headinf[4];
			inScan    = headinf[5];
			noParams  = headinf[6];
			sentCfg   = headinf[7];

			TRITECH_MSG_EXPECT_BYTE_AT(0x0A, 22);
		};

		void print()
		{
			printf("mtAliveMsg: node = %#x headTime_msec = %d motorPos = %d inCentre = %d centered = %d motoring = %d motorOn"
					"= %d dir = %d inScan = %d noParams = %d sentCfg = %d\n", txNode, headTime_msec, motorPos, inCentre, centered,
					motoring, motorOn, dir, inScan, noParams, sentCfg);
		}
	};



	// ######################################################################
	struct mtHeadDataMsg
	{
		uint8_t packetSequence; //!< If this is part of a multi-packet sequence, which packet is this?
		bool isLastInSequence; //!< Is this the last packet in the sequence?
		uint8_t txNode;

		//! Head Status Data
		/*! {@ */
		bool hdPwrLoss;          //!< Head is in reset condition
		bool motorErr;           //!< Motor has lost sync, re-send parameters.
		bool dataRangeis0to80db; //!< When in 8-bit adc datamode, data is 0..255 = 0..80db
		bool messageAppended;    //!< Message appended after last packet data reply
		/*! @}*/

		enum sweepCode_t {Scanning_Normal, Scan_AtLeftLimit, Scan_AtRightLimit, Scan_AtCentre};
		sweepCode_t sweepCode;

		//! Head Control
		/*! @{ */
		struct headControl_t
		{
			bool adc8on;
			bool cont;
			bool scanright;
			bool invert;
			bool motoroff;
			bool txoff;
			bool spare;
			bool chan2;
			bool raw;
			bool hasMotor;
			bool applyOffset;
			bool pingPong;
			bool starteLLim;
			bool replyASL;
			bool replyThr;
			bool ignoreSensor;
		} headControl;
		/*! @} */

		int rangeScale;
		enum rangeUnits_t {feet, fathoms, yards};
		rangeUnits_t rangeUnits;

		float stepSize_degrees;
		float bearing_degrees; //!< The current bearing of the sonar head


		mtHeadDataMsg(Message const& msg)
		{
			TRITECH_MSG_EXPECT_BYTE_AT('@', 1);
			TRITECH_MSG_EXPECT_BYTE_AT(mtHeadData, 11);
			uint8_t const msgSequenceBitset = msg.data[12];
			packetSequence   = msgSequenceBitset & 0xEF;
			isLastInSequence = msgSequenceBitset & 0x80;

			uint16_t const totalByteCount = msg.data[14] | (msg.data[15] << 8);

			// We expect the device type to be '11', i.e. a Digital Img Sonar (hDIG).
			TRITECH_MSG_EXPECT_BYTE_AT(0x0A, 16);

			std::bitset<8> headStatus = msg.data[17];
			hdPwrLoss          = headStatus[0];
			motorErr           = headStatus[1];
			dataRangeis0to80db = headStatus[4];
			messageAppended    = headStatus[7];

			switch(msg.data[18])
			{
				case 0: sweepCode = Scanning_Normal; break;
				case 1: sweepCode = Scan_AtLeftLimit; break;
				case 2: sweepCode = Scan_AtRightLimit; break;
				case 5: sweepCode = Scan_AtCentre; break;
				default:
								std::cerr << __FUNCTION__ << ":" << __LINE__ <<
									" - Unknown Sweep Code (" << msg.data[18] << ")" << std::endl;
								break;
			}

			std::bitset<16> headCtrl = msg.data[19] | (uint16_t(msg.data[20]) << 8);
			headControl.adc8on       = headCtrl[0];          
      headControl.cont         = headCtrl[1];                     
      headControl.scanright    = headCtrl[2];                           
      headControl.invert       = headCtrl[3];                                    
      headControl.motoroff     = headCtrl[4];                                           
      headControl.txoff        = headCtrl[5];                                                     
      headControl.spare        = headCtrl[6];                                                               
      headControl.chan2        = headCtrl[7];                                                                         
      headControl.raw          = headCtrl[8];
      headControl.hasMotor     = headCtrl[9];
      headControl.applyOffset  = headCtrl[10];
      headControl.pingPong     = headCtrl[11];
      headControl.starteLLim   = headCtrl[12];
      headControl.replyASL     = headCtrl[13];
      headControl.replyThr     = headCtrl[14];
      headControl.ignoreSensor = headCtrl[15];

			int rangeScale = ((uint16_t(msg.data[21]) | (uint16_t(msg.data[22]) << 8)) & 0xDFFF)*10;
			uint8_t rangeTp = uint8_t(msg.data[22]) >> 6;
			switch(rangeTp)
			{
				case 1: rangeUnits = feet; break;
				case 2: rangeUnits = fathoms; break;
				case 3: rangeUnits = yards; break;
				default:
								std::cerr << __FUNCTION__ << ":" << __LINE__ <<
									" - Unknown range units Code (" << rangeTp << ")" << std::endl;
								break;
			}

			stepSize_degrees = msg.data[40]/16.0 * 180.0/200.0;
			bearing_degrees  = (uint16_t(msg.data[41]) | (uint16_t(msg.data[42]) << 8))/16.0 * 180.0/200.0;



		}




	};
	
}

#endif // TRITECHMICRON_MESSAGETYPES_H

