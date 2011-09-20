#ifndef BASE_LIBS_BASE_LIBS_PUBLISHER_POLICY_H_
#define BASE_LIBS_BASE_LIBS_PUBLISHER_POLICY_H_

#include <base_libs/policy.h>
#include <base_libs/multi_publisher.h>

namespace base_libs
{

template<class __Publisher = ros::Publisher>
class PublisherPolicy : public Policy
{
protected:
	ros::MultiPublisher<__Publisher> publishers_;
	
public:
	PublisherPolicy( ros::NodeHandle & nh )
	:
		Policy( nh )
	{
		ROS_INFO( "Creating publisher policy..." );
		ROS_INFO( "Done creating publisher policy." );
	}
};

}

#endif // BASE_LIBS_BASE_LIBS_PUBLISHER_POLICY_H_
