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
