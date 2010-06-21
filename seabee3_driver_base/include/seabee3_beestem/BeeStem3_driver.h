#include "seabee3_beestem/BeeStem3.h"
#include <vector>
#include <map>

#define HEADING_K 0
#define HEADING_P 15
#define HEADING_I 0
#define HEADING_D 0

#define DEPTH_K 0
#define DEPTH_P 33
#define DEPTH_I 0
#define DEPTH_D 0

namespace Actions
{
	const static int NONE = -1;
	const static int AXIS_INVERT = 0;
	const static int DIVE = 1;
	const static int SURFACE = 2;
	const static int STRAFE = 3;
	const static int SPEED = 4;
	const static int HEADING = 5;
	const static int ARM_NEXT_DEV = 6;
	const static int FIRE_DEV = 7;
}

class BeeStem3Driver
{
public:

	struct FiringDeviceID
	{
		const static int Shooter = 0;
		const static int DropperStage1 = 1;
		const static int DropperStage2 = 2;
	};

	struct BeeStemFlags
	{
		bool initFlag;

		BeeStemFlags()
		{
			initFlag = false;
		}
	};
	
	BeeStem3Driver(unsigned int usbIndex);
	
	~BeeStem3Driver();
	
	void step();
	
	void readPressure(int & intlPressure, int & extPressure);

	void fireDevice(int deviceID);
	
	BeeStem3 * itsStem;

//	static void getMotorControllerMsg(RobotSimEvents::BeeStemMotorControllerMessagePtr & msg, int mc0, int mc1, int mc2, int mc3, int mc4, int mc5, int mc6, int mc7, int mc8);
	
	bool dropper1_ready;
	bool dropper2_ready;
	bool shooter_ready;

private:

	void initPose();

	//  nub::soft_ref<Serial> itsKillSwitch;
	//  pthread_mutex_t itsKillSwitchLock;

	std::map<int, int> itsJSMappings;
	//std::vector<int> itsJSValues;
	std::vector<int> itsButValues;
	int itsDesiredHeading, itsDesiredDepth, itsDesiredSpeed;
	int itsUpdateHeading, itsUpdateDepth, itsUpdateSpeed;
	int itsLastUpdateHeading, itsLastUpdateDepth, itsLastUpdateSpeed;
	int mShooterState;
	int mDropperState;
	int mFiringDeviceID;
	
	unsigned int mUsbIndex;

	BeeStemFlags mFlags;
};
