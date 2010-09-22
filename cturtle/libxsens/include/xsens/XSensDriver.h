#ifndef XSensDriver_H
#define XSensDriver_H

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <signal.h>
#include <string>

#include <xsens/cmtdef.h>
#include <xsens/xsens_time.h>
#include <xsens/xsens_list.h>
#include <xsens/cmtscan.h>
#include <xsens/cmt3.h>

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

	XSensDriver( std::string port = "" );

	~XSensDriver();

	bool updateData();

	bool initMe();
	int doHardwareScan( xsens::Cmt3 &, CmtDeviceId[] );
	void doMtSettings( xsens::Cmt3 &, CmtOutputMode &, CmtOutputSettings &, CmtDeviceId[] );

	Vector3 accel_;
	Vector3 gyro_;
	Vector3 mag_;
	Vector3 ori_;

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

	std::string port_;

	xsens::Packet * packet;

	bool inited;
};

#endif
