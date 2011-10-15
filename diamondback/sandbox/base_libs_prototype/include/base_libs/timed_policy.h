#ifndef BASE_LIBS_BASE_LIBS_TIMED_POLICY_H_
#define BASE_LIBS_BASE_LIBS_TIMED_POLICY_H_

#include <base_libs/updateable_policy.h>
#include <ros/time.h>

namespace base_libs
{

BASE_LIBS_DECLARE_POLICY( Timed, UpdateablePolicy )

BASE_LIBS_DECLARE_POLICY_CLASS( Timed )
{
	BASE_LIBS_MAKE_POLICY_NAME( Timed )

protected:
	ros::Time last_time_;
	ros::Time now_;
	double dt_;
	
	BASE_LIBS_DECLARE_POLICY_CONSTRUCTOR( Timed ),
		dt_( 0 )
	{
		printPolicyActionStart( "create", this );
		printPolicyActionDone( "create", this );
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
