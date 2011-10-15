#ifndef BASE_LIBS_BASE_LIBS_TIMED_POLICY_H_
#define BASE_LIBS_BASE_LIBS_TIMED_POLICY_H_

#include <base_libs/updateable_policy.h>
#include <ros/time.h>

namespace base_libs
{

class TimedPolicy : public UpdateablePolicy
{
protected:
	ros::Time last_time_;
	ros::Time now_;
	
public:
	TimedPolicy( ros::NodeHandle & nh )
	:
		UpdateablePolicy( nh )
	{
		PRINT_INFO( "Creating timed policy..." );
		PRINT_INFO( "Done creating timed policy." );
	}
	
	BASE_LIBS_ENABLE_UPDATE
	{
		last_time_ = now_;
		now_ = ros::Time::now();
		dt_ = ( now_ - last_time_ ).toSec();
	}
};

}

#endif // BASE_LIBS_BASE_LIBS_TIMED_POLICY_H_
