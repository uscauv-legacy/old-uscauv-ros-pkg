#include <base_libs/macros.h>
#include <base_libs/node.h>
#include <base_libs/tf_tranceiver_policy.h>

class TestTfTranceiverPolicy : public base_libs::Node<base_libs::TfTranceiverPolicy >
{
public:
	ros::Time last_time_;
	ros::Time now_;

	TestTfTranceiverPolicy( ros::NodeHandle & nh ) : base_libs::Node<base_libs::TfTranceiverPolicy >( nh ), last_time_( ros::Time::now() ), now_( ros::Time::now() )
	{
		//
	}
	
	void spinFirst()
	{
		tf::Transform transform( tf::Quaternion( 0, 0, 0, 1 ), tf::Vector3( 0, 0, 0 ) );
		publishTransform( transform, "/world", "/frame1", now_ );
		publishTransform( transform, "/world", "/frame2", now_ );
		
		lookupTransform( "/world", "/frame1" );
		lookupTransform( "/world", "/frame2" );
		
		lookupTransform( "/world", "/frame1", now_ - ros::Duration( 5 ) );
		lookupTransform( "/world", "/frame2", now_ - ros::Duration( 5 ) );
	}
	
	void spinOnce()
	{
		now_ = ros::Time::now();
		
		auto frame1_last_to_frame2_past( lookupTransform( "/frame1", last_time_, "/frame2", last_time_ - ros::Duration( 1 ), "/world" ) );
		frame1_last_to_frame2_past.child_frame_id_ = "/frame2_past";
		
		publishTransform( frame1_last_to_frame2_past, now_ );
		
		// get the last state of /world -> /frame1
		auto world_to_frame1_last( lookupTransform( "/world", "/frame1", last_time_ ) );
		// get the last state of /world -> /frame1
		auto world_to_frame2_last( lookupTransform( "/world", "/frame2", last_time_ ) );
		
		world_to_frame1_last.getOrigin().setX( world_to_frame1_last.getOrigin().x() +  1 * ( now_ - last_time_ ).toSec() );
		world_to_frame2_last.getOrigin().setY( world_to_frame2_last.getOrigin().y() +  1 * ( now_ - last_time_ ).toSec() );
		
		// update /world -> frame1 at the current time
		publishTransform( world_to_frame1_last, now_ );
		// update /world -> frame2 at the current time
		publishTransform( world_to_frame2_last, now_ );
		
		last_time_ = now_;
	}
};

BASE_LIBS_DECLARE_NODE( TestTfTranceiverPolicy, "test_tf_tranceiver_policy" )
