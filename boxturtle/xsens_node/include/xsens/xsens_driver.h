/*******************************************************************************
 *
 *      xsens_driver
 * 
 *      Copyright (c) 2010, Edward T. Kaszubski (ekaszubski@gmail.com)
 *      All rights reserved.
 *
 *      Redistribution and use in source and binary forms, with or without
 *      modification, are permitted provided that the following conditions are
 *      met:
 *      
 *      * Redistributions of source code must retain the above copyright
 *        notice, this list of conditions and the following disclaimer.
 *      * Redistributions in binary form must reproduce the above
 *        copyright notice, this list of conditions and the following disclaimer
 *        in the documentation and/or other materials provided with the
 *        distribution.
 *      * Neither the name of the USC Underwater Robotics Team nor the names of its
 *        contributors may be used to endorse or promote products derived from
 *        this software without specific prior written permission.
 *      
 *      THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *      "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *      LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *      A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *      OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *      SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *      LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *      DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *      THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *      (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *      OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *******************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <signal.h>

#include "xsens/cmtdef.h"
#include "xsens/xsens_time.h"
#include "xsens/xsens_list.h"
#include "xsens/cmtscan.h"
#include "xsens/cmt3.h"

#ifndef XSensDriver_H
#define XSensDriver_H

class XSensDriver
{
public:

	struct Mode
	{
		const static int quat = 0;
		const static int euler = 1;
		const static int cos_mat = 2;
	};
	
	struct Vector3
	{
		double x;
		double y;
		double z;
	};
	
	XSensDriver(unsigned int usbIndex);
	
	~XSensDriver();

	bool updateData();

	bool initMe();
	int doHardwareScan(xsens::Cmt3 &, CmtDeviceId []);
	void doMtSettings(xsens::Cmt3 &, CmtOutputMode &, CmtOutputSettings &, CmtDeviceId []);
	//void getUserInputs(CmtOutputMode &, CmtOutputSettings &);

	Vector3 accel;
	Vector3 gyro;
	Vector3 mag;
	Vector3 ori;

private:
	xsens::Cmt3 cmt3;
	CmtDeviceId deviceIds[256];

	CmtOutputMode mode;
	CmtOutputSettings settings;

	XsensResultValue res;

	CmtCalData caldata;
	CmtQuat qat_data;
	CmtEuler euler_data;
	CmtMatrix matrix_data;
	double tdata;
	
	unsigned int mUsbIndex;
	
	xsens::Packet * packet;

	bool inited;
};

#endif
