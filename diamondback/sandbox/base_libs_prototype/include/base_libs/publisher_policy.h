#ifndef BASE_LIBS_BASE_LIBS_PUBLISHER_POLICY_H_
#define BASE_LIBS_BASE_LIBS_PUBLISHER_POLICY_H_

#include <base_libs/policy.h>
#include <base_libs/multi_publisher.h>
#include <base_libs/type_utils.h>

namespace base_libs
{

template<class __Publisher = ros::Publisher>
class PublisherPolicy : public Policy
{
private:
	ros::NodeHandle nh_rel_;
	
protected:
	ros::MultiPublisher<__Publisher> publishers_;
	
public:
	PublisherPolicy( ros::NodeHandle & nh )
	:
		Policy(),
		nh_rel_( nh )
	{
		ROS_INFO( "Creating publisher policy..." );
		ROS_INFO( "Done creating publisher policy." );
	}
};

}

#endif // BASE_LIBS_BASE_LIBS_PUBLISHER_POLICY_H_
