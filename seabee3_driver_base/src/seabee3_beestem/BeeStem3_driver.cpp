#include "seabee3_beestem/BeeStem3_driver.h"

#include <iostream>
#include <stdlib.h>
#include <sstream>

using namespace std;

BeeStem3Driver::BeeStem3Driver(unsigned int usbIndex)
{
	mUsbIndex = usbIndex;
	stringstream portURI;
	portURI << "/dev/ttyUSB" << mUsbIndex;
	itsStem = new BeeStem3(portURI.str());
	itsUpdateHeading = 0;
	itsUpdateDepth = 0;
	itsUpdateSpeed = 0;
	itsLastUpdateHeading = 0;
	itsLastUpdateDepth = 0;
	itsLastUpdateSpeed = 0;
	
	//itsKillSwitch->configure("/dev/ttyUSB1",57600);
	mFlags.initFlag = false;
	//itsJSValues.resize(8);
	itsButValues.resize(20);
	// init trigger values
	itsJSMappings[Actions::SURFACE] = -100;
	itsJSMappings[Actions::DIVE] = -100;
	//itsJSValues[4] = -100; //this only works if those axes are properly mapped
	//itsJSValues[5] = -100;
	
	dropper1_ready = true;
	dropper2_ready = true;
	shooter_ready = true;
}

// ######################################################################
BeeStem3Driver::~BeeStem3Driver()
{
}

// ######################################################################
void BeeStem3Driver::initPose()
{
	mFlags.initFlag = true;

	//itsStemMutex.lock();
	itsStem->setPID(0, HEADING_K, HEADING_P, HEADING_I, HEADING_D);
	itsStem->setPID(1, DEPTH_K, DEPTH_P, DEPTH_I, DEPTH_D);

	int accelX, accelY, accelZ;
	int compassHeading, compassPitch, compassRoll;
	int internalPressure, externalPressure;
	int headingK, headingP, headingD, headingI, headingOutput;
	int depthK, depthP, depthD, depthI, depthOutput;
	char killSwitch;

	bool successful = itsStem->getSensors(accelX, accelY, accelZ, compassHeading, compassPitch, compassRoll, internalPressure, externalPressure, itsDesiredHeading, itsDesiredDepth, itsDesiredSpeed, headingK, headingP, headingD, headingI, headingOutput, depthK, depthP, depthD, depthI, depthOutput, killSwitch);

	if (successful)
	{
		itsUpdateHeading = compassHeading;
		itsLastUpdateHeading = itsUpdateHeading;
		itsStem->setDesiredHeading(compassHeading);
		itsUpdateDepth = externalPressure;
		itsLastUpdateDepth = itsUpdateDepth;
		itsStem->setDesiredDepth(externalPressure);
	}

	//itsStemMutex.unlock();
}

void BeeStem3Driver::readPressure(int & intlPressure, int & extPressure)
{
	int accelX, accelY, accelZ, compassHeading, compassPitch, compassRoll,desiredHeading, desiredDepth, desiredSpeed,headingK, headingP, headingD, headingI, headingOutput,  depthK, depthP, depthD, depthI, depthOutput;
	char killSwitc;
    
	itsStem->getSensors(accelX, accelY, accelZ, compassHeading, compassPitch, compassRoll, intlPressure, extPressure,desiredHeading, desiredDepth, desiredSpeed,headingK, headingP, headingD, headingI, headingOutput,  depthK, depthP, depthD, depthI, depthOutput, killSwitc);
	
}

void BeeStem3Driver::step()
{
	if (!mFlags.initFlag)
		initPose();

	int accelX, accelY, accelZ;
	int compassHeading, compassPitch, compassRoll;
	int internalPressure, externalPressure;
	int headingK, headingP, headingD, headingI, headingOutput;
	int depthK, depthP, depthD, depthI, depthOutput;
	char killSwitch;
	//  int thruster1,  thruster2,  thruster3,  thruster4,  thruster5,  thruster6;

	//itsStemMutex.lock();
	bool successful = itsStem->getSensors(accelX, accelY, accelZ, compassHeading, compassPitch, compassRoll, internalPressure, externalPressure, itsDesiredHeading, itsDesiredDepth, itsDesiredSpeed, headingK, headingP, headingD, headingI, headingOutput, depthK, depthP, depthD, depthI, depthOutput, killSwitch);

	//itsUpdateMutex.lock();
	int tempHeading = itsUpdateHeading;
	int tempDepth = itsUpdateDepth;
	int tempSpeed = itsUpdateSpeed;
	//itsUpdateMutex.unlock();

	if (itsLastUpdateHeading != tempHeading)
	{
		itsLastUpdateHeading = tempHeading;
		itsStem->setDesiredHeading(tempHeading);
	}
	if (itsLastUpdateDepth != tempDepth)
	{
		itsLastUpdateDepth = tempDepth;
		itsStem->setDesiredDepth(tempDepth);
	}
	if (itsLastUpdateSpeed != tempSpeed)
	{
		itsLastUpdateSpeed = tempSpeed;
		itsStem->setDesiredSpeed(tempSpeed);
	}

	//itsStemMutex.unlock();
	/*if (successful)
	{

		RobotSimEvents::BeeStemMessagePtr msg = new RobotSimEvents::BeeStemMessage;

		msg->accelX = accelX;
		msg->accelY = accelY;
		msg->accelZ = accelZ;
		msg->compassHeading = compassHeading;
		msg->compassPitch = compassPitch;
		msg->compassRoll = compassRoll;
		msg->internalPressure = internalPressure;
		msg->externalPressure = externalPressure;
		msg->desiredHeading = itsDesiredHeading;
		msg->desiredDepth = itsDesiredDepth;
		msg->desiredSpeed = itsDesiredSpeed;
		msg->headingK = headingK;
		msg->headingP = headingP;
		msg->headingD = headingD;
		msg->headingI = headingI;
		msg->headingOutput = headingOutput;
		msg->depthK = depthK;
		msg->depthP = depthP;
		msg->depthD = depthD;
		msg->depthI = depthI;
		msg->depthOutput = depthOutput;
		msg->killSwitch = killSwitch;

		this->publish("BeeStemMessageTopic", msg);

		//       char* str = new char[32];
		//       sprintf(str, "\n%04d", externalPressure);

		//       itsKillSwitch->write(str,32);

		//       delete str;
	}
	else if (!successful)
		LINFO("Error reading from BeeStem");
	*/
}

void BeeStem3Driver::fireDevice(int deviceID)
{
	switch(deviceID)
	{
	case FiringDeviceID::Shooter:
		cout << "Firing torpedo!" << endl;
		itsStem->setThruster(BeeStem3::MotorControllerIDs::SHOOTER, 95);
		usleep(50 * 1000);
		itsStem->setThruster(BeeStem3::MotorControllerIDs::SHOOTER, 0);
		shooter_ready = false;
		break;
	case FiringDeviceID::DropperStage1:
		cout << "Dropping first marker!" << endl;
		itsStem->setThruster(BeeStem3::MotorControllerIDs::DROPPER_STAGE1, 95);
		usleep(50 * 1000);
		itsStem->setThruster(BeeStem3::MotorControllerIDs::DROPPER_STAGE1, 0);
		dropper1_ready = false;
		break;
	case FiringDeviceID::DropperStage2:
		cout << "Dropping second marker!" << endl;
		itsStem->setThruster(BeeStem3::MotorControllerIDs::DROPPER_STAGE2, 95);
		usleep(50 * 1000);
		itsStem->setThruster(BeeStem3::MotorControllerIDs::DROPPER_STAGE2, 0);
		dropper2_ready = false;
		break;
	}
}
	// Get a XBox360RemoteControl Message
	/*if (eMsg->ice_isA("::RobotSimEvents::JoyStickControlMessage"))
	{
		RobotSimEvents::JoyStickControlMessagePtr msg = RobotSimEvents::JoyStickControlMessagePtr::dynamicCast(eMsg);
		LINFO("Got message %d %s %d",msg->axis,msg->axisName.c_str(), msg->axisVal);

		itsJSMappings[Actions::toInt[msg->axisName]] = msg->axisVal;

		static bool deviceCycleBtnWaitingReset = false; //prevent it from cycling really f-ing fast through devices
		if(Actions::toInt[msg->axisName] == Actions::ARM_NEXT_DEV)
		{
			if(msg->axisVal > 0 && !deviceCycleBtnWaitingReset)
			{
				mFiringDeviceID = mFiringDeviceID >= FiringDeviceID::MAX ? FiringDeviceID::Null : mFiringDeviceID + 1;
				deviceCycleBtnWaitingReset = true;
			}
			else if(msg->axisVal < 0 && !deviceCycleBtnWaitingReset)
			{
				mFiringDeviceID = mFiringDeviceID <= FiringDeviceID::Null ? FiringDeviceID::MAX : mFiringDeviceID - 1;
				deviceCycleBtnWaitingReset = true;
			}
			else if(msg->axisVal == 0 && deviceCycleBtnWaitingReset)
			{
				deviceCycleBtnWaitingReset = false;
				printf(mFiringDeviceID == FiringDeviceID::Null ? "All devices set to be disarmed\n" : "Device %d set to be armed\n", mFiringDeviceID);
			}
		}

		if (msg->button >= 0)
		{
			itsButValues[msg->button] = msg->butVal;
		}

		mFlags.needsUpdateFromJoystick = true;
	}*/
	// Get a BeeStemConfig Message
	/*if (eMsg->ice_isA("::RobotSimEvents::BeeStemConfigMessage"))
	{
		RobotSimEvents::BeeStemConfigMessagePtr msg = RobotSimEvents::BeeStemConfigMessagePtr::dynamicCast(eMsg);

		//      int h,d,s;
		if (msg->deviceToFire && msg->deviceToFire != FiringDeviceID::Null) //so...yeah we can't fire a device and update PID at the same time, NBD
		{
			fireDevice(msg->deviceToFire);
		}
		else if (msg->enablePID == 1)
		{
			itsStemMutex.lock();
			if (msg->enableVal == 1)
				itsStem->setPID(3, msg->headingK, msg->headingP, msg->headingI, msg->headingD);
			else
				itsStem->setPID(2, msg->headingK, msg->headingP, msg->headingI, msg->headingD);
			itsStemMutex.unlock();
		}
		else if (msg->updateHeadingPID == 0 && msg->updateDepthPID == 0)
		{
			itsUpdateMutex.lock();

			LINFO("Update Pose: %d\n",msg->updateDesiredValue);
			if (msg->updateDesiredValue == 1)
				itsUpdateHeading = msg->desiredHeading;
			else if (msg->updateDesiredValue == 2)
				itsUpdateDepth = msg->desiredDepth;
			else if (msg->updateDesiredValue == 3)
				itsUpdateSpeed = msg->desiredSpeed;

			itsUpdateMutex.unlock();
		}
		else if (msg->updateDepthPID == 1)
		{
			itsStemMutex.lock();
			itsStem->setPID(0, msg->depthK, msg->depthP, msg->depthI, msg->depthD);
			itsStemMutex.unlock();
		}
		else if (msg->updateHeadingPID == 1)
		{
			itsStemMutex.lock();
			itsStem->setPID(1, msg->headingK, msg->headingP, msg->headingI, msg->headingD);
			itsStemMutex.unlock();
		}
	}*/

/*void BeeStem3Driver::getMotorControllerMsg(RobotSimEvents::BeeStemMotorControllerMessagePtr & msg, int mc0, int mc1, int mc2, int mc3, int mc4, int mc5, int mc6, int mc7, int mc8)
{
	vector<int> values;
	values.resize(BeeStem3::NUM_MOTOR_CONTROLLERS);
	values[BeeStem3::MotorControllerIDs::FWD_LEFT_THRUSTER] = mc0;
	values[BeeStem3::MotorControllerIDs::FWD_RIGHT_THRUSTER] = mc1;
	values[BeeStem3::MotorControllerIDs::STRAFE_BACK_THRUSTER] = mc2;
	values[BeeStem3::MotorControllerIDs::STRAFE_FRONT_THRUSTER] = mc3;
	values[BeeStem3::MotorControllerIDs::DEPTH_LEFT_THRUSTER] = mc4;
	values[BeeStem3::MotorControllerIDs::DEPTH_RIGHT_THRUSTER] = mc5;
	values[BeeStem3::MotorControllerIDs::SHOOTER] = mc6;
	values[BeeStem3::MotorControllerIDs::DROPPER_STAGE1] = mc7;
	values[BeeStem3::MotorControllerIDs::DROPPER_STAGE2] = mc8;
	msg->values = values;
}*/
