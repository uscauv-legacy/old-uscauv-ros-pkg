#ifndef BASE_LIBS_BASE_LIBS_SERVICE_SERVER_POLICY_H_
#define BASE_LIBS_BASE_LIBS_SERVICE_SERVER_POLICY_H_

#include <base_libs/policy.h>
#include <ros/service_server.h>
#include <ros/node_handle.h>

namespace base_libs
{

class ServiceServerPolicy : public Policy
{
private:
	ros::NodeHandle nh_rel_;
	
public:
	ServiceServerPolicy( ros::NodeHandle & nh )
	:
		Policy(),
		nh_rel_( nh )
	{
		PRINT_INFO( "Creating service server policy..." );
		PRINT_INFO( "Done creating service server policy." );
	}
	
};

}

#endif // BASE_LIBS_BASE_LIBS_SERVICE_SERVER_POLICY_H_
