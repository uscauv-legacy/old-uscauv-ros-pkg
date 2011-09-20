#ifndef BASE_LIBS_BASE_LIBS_SUBSCRIBER_POLICY_H_
#define BASE_LIBS_BASE_LIBS_SUBSCRIBER_POLICY_H_

// NOTE: multi_subscriber needs to be implemented first

#include <base_libs/policy.h>
#include <base_libs/multi_subscriber.h>

namespace base_libs
{

class SubscriberPolicy : public Policy
{
protected:
	//ros::MultiSubscriber multi_pub_;
	
public:
	SubscriberPolicy( ros::NodeHandle & nh )
	:
		Policy( nh )
	{
		ROS_INFO( "Creating subscriber policy..." );
		ROS_INFO( "Done creating subscriber policy." );
	}
};

}

#endif // BASE_LIBS_BASE_LIBS_SUBSCRIBER_POLICY_H_
