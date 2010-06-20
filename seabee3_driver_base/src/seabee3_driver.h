#include "libseabee3/BeeStem3.h"
#include "seabee3_driver_base/MotorCntl.h"
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

namespace XBox360RemoteControl
{
	struct Keys
	{
		struct Actions
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
		};
	};
}

class seabee3_driver
{
public:

	struct FiringDeviceID
	{
		const static int Null = -1;
		const static int MIN = 0;
		const static int Shooter = 0;
		const static int DropperStage1 = 1;
		const static int DropperStage2 = 2;
		const static int MAX = 2;
	};

	struct ShooterState
	{
		const static int Idle = 0;
		const static int Armed = 1;
		const static int Firing = 2;
		const static int Reset = 3;
		const static int MAX = 3;
	};

	/*
	 * Ensures proper functioning of the dropper;
	 * -Each stage must be armed prior to dropping
	 * -Stage2 can't be armed until Stage1 has been reset
	 */
	struct DropperState
	{
		const static int AllIdle = 0;
		const static int Stage1Armed = 1;
		const static int Stage1Dropping = 2;
		const static int Stage1Reset = 3;
		const static int Stage1Idle = 4;
		const static int Stage2Armed = 5;
		const static int Stage2Dropping = 6;
		const static int Stage2Reset = 7;
		const static int MAX = 7;
	};

	struct BeeStemFlags
	{
		bool needsUpdateFromJoystick;
		bool initFlag;

		BeeStemFlags()
		{
			needsUpdateFromJoystick = false;
			initFlag = false;
		}
	};
	
	seabee3_driver();
	
	~seabee3_driver();
	
	void step();
	
	void readPressure(int & intlPressure, int & extPressure);

	void fireDevice(int deviceID);
	
	BeeStem3 * itsStem;
	
	void setValuesFromJoystick();

//	static void getMotorControllerMsg(RobotSimEvents::BeeStemMotorControllerMessagePtr & msg, int mc0, int mc1, int mc2, int mc3, int mc4, int mc5, int mc6, int mc7, int mc8);

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

	BeeStemFlags mFlags;
};
