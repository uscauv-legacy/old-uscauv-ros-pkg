#ifndef BASE_LIBS_BASE_LIBS_RUNABLE_POLICY_H_
#define BASE_LIBS_BASE_LIBS_RUNABLE_POLICY_H_

#include <base_libs/node_handle_policy.h>
#include <ros/rate.h>

namespace base_libs
{


BASE_LIBS_DECLARE_POLICY( Runable, NodeHandlePolicy )

BASE_LIBS_DECLARE_POLICY_CLASS( Runable )
{
	BASE_LIBS_MAKE_POLICY_NAME( Runable )
	
protected:
	ros::Rate * loop_rate_;
	bool run_;
	
	BASE_LIBS_DECLARE_POLICY_CONSTRUCTOR( Runable ),
		run_( false )
	{
		printPolicyActionStart( "create", this );
		
		loop_rate_ = new ros::Rate( ros::ParamReader<double, 1>::readParam( NodeHandlePolicy::nh_rel_, "loop_rate", 10 ) );
		
		printPolicyActionDone( "create", this );
	}
	
	~RunablePolicy()
	{
		if( loop_rate_ ) delete loop_rate_;
	}
	
	virtual void spinFirst(){}
	virtual void spinOnce(){}
	
	virtual void spin()
	{
		run_ = true;
		
		PRINT_INFO( "----- Spinning... -----" );
		
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

#endif // BASE_LIBS_BASE_LIBS_RUNABLE_POLICY_H_
