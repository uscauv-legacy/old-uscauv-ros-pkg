#ifndef BASE_LIBS_BASE_LIBS_POLICY_H_
#define BASE_LIBS_BASE_LIBS_POLICY_H_

#include <ros/node_handle.h>
#include <ros/rate.h>
#include <base_libs/param_reader.h>

namespace base_libs
{

class Policy
{
protected:
	ros::NodeHandle nh_rel_;
	ros::Rate * loop_rate_;
	bool run_;
	
public:
	Policy( ros::NodeHandle & nh )
	:
		nh_rel_( nh ),
		run_( false )
	{
		ROS_INFO( "Creating basic policy..." );
		
		loop_rate_ = new ros::Rate( ros::ParamReader<double, 1>::readParam( nh, "loop_rate", 10 ) );
		
		ROS_INFO( "Done creating basic policy." );
	}
	
	virtual void spinFirst(){}
	virtual void spinOnce() = 0;
	
	virtual void spin()
	{
		run_ = true;
		
		spinFirst();
		
		while( run_ && ros::ok() )
		{
			spinOnce();
			ros::spinOnce();
			if( loop_rate_ ) loop_rate_->sleep();
		}
	}
	
	virtual void interrupt()
	{
		run_ = false;
	}
};

}

#endif // BASE_LIBS_BASE_LIBS_POLICY_H_
