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
			TRITECH_MSG_LENGTH_CHK(13)
			TRITECH_MSG_EXPECT_BYTE_AT(0x80, 0);

			node = msg.data[1];

			softwareVersion = msg.data[2];

			infoBits = msg.data[3];

			uid =  uint16_t(msg.data[4] << 0);
			uid |= uint16_t(msg.data[5] << 8);

			programLength  = uint32_t(msg.data[6] << 0);
			programLength |= uint32_t(msg.data[7] << 8);
			programLength |= uint32_t(msg.data[8] << 16);
			programLength |= uint32_t(msg.data[9] << 24);

			checksum  = uint32_t(msg.data[10]  << 0);
			checksum += uint32_t(msg.data[11] << 8);
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
		uint8_t node;
		int headTime_msec;
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
			TRITECH_MSG_LENGTH_CHK(10)
			TRITECH_MSG_EXPECT_BYTE_AT(0x80, 0);

			node = msg.data[1];

			TRITECH_MSG_EXPECT_BYTE_AT(0x80, 2);

			uint32_t htms = 0;
			htms  = uint32_t(msg.data[3]) << 0;
			htms |= uint32_t(msg.data[4]) << 8;
			htms |= uint32_t(msg.data[5]) << 16;
			htms |= uint32_t(msg.data[6]) << 24;
			headTime_msec = htms;

			motorPos  = msg.data[7] << 0;
			motorPos |= msg.data[8] << 8;

			std::bitset<8> headinf = msg.data[9];
			inCentre  = headinf[0];
			centered  = headinf[1];
			motoring  = headinf[2];
			motorOn   = headinf[3];
			dir       = headinf[4];
			inScan    = headinf[5];
			noParams  = headinf[6];
			sentCfg   = headinf[7];
	};

	void print()
	{
		printf("mtAliveMsg: node = %#x headTime_msec = %d motorPos = %d inCentre = %d centered = %d motoring = %d motorOn"
				"= %d dir = %d inScan = %d noParams = %d sentCfg = %d\n", node, headTime_msec, motorPos, inCentre, centered,
				motoring, motorOn, dir, inScan, noParams, sentCfg);
	}

};
}

#endif // TRITECHMICRON_MESSAGETYPES_H

