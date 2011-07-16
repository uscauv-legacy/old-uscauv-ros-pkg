#include <base_node/base_node.h>
#include <geometry_msgs/Twist.h>
#include <common_utils/math.h>
#include <common_utils/tf.h>

// msgs
#include <seabee3_driver_base/KillSwitch.h>

// srvs
#include <seabee3_common/SetDesiredPose.h> // for SetDesiredPose
#include <std_srvs/Empty.h> // for ResetPose
#include <thread>

class CompetitionDemo : public BaseNode<>
{

protected:
	geometry_msgs::Twist cmd_vel_;
	ros::Publisher cmd_vel_pub_;
	ros::Subscriber kill_switch_sub_;
	ros::ServiceClient set_desired_pose_cli_;
	ros::ServiceClient reset_pose_cli_;

	tf::Transform current_pose_;

	bool kill_timers_;
	bool kill_behaviors_;
	bool running_;

  enum FSMState_t {waiting_for_start, running, killing};
  FSMState_t state_;
  std::thread FSMthread_;

  ////////////////////////////////
  // Run Parameters
  ////////////////////////////////
  double error_threshold_;
  double depth_;
  double distance_to_gate_;

public:
  // ######################################################################
	CompetitionDemo( ros::NodeHandle & nh ) :
		BaseNode<> ( nh ), kill_timers_( false ), kill_behaviors_( false ), running_( false )
	{
		nh_local_.param( "error_threshold", error_threshold_, 0.1 );
		nh_local_.param( "depth", depth_, 1.7 );
		nh_local_.param( "distance_to_gate", distance_to_gate_, 10.0 );

		kill_switch_sub_ = nh_local_.subscribe( "/seabee3/kill_switch", 2, &CompetitionDemo::killSwitchCB, this );
		set_desired_pose_cli_ = nh_local_.serviceClient<seabee3_common::SetDesiredPose> ( "/seabee3/set_desired_pose" );
		reset_pose_cli_ = nh_local_.serviceClient<std_srvs::Empty> ( "/seabee3/reset_pose" );

    reset();
	}

  // ######################################################################
	void reset()
	{
    running_ = false;
		kill_behaviors_ = false;
		ros::Duration( 0.1 ).sleep();
		std_srvs::Empty reset_request;
		reset_pose_cli_.call( reset_request );
	}

  // ######################################################################
	void killSwitchCB( const seabee3_driver_base::KillSwitch::ConstPtr & kill_switch_msg )
	{
		static bool last_killed_state_ = kill_switch_msg->is_killed;
    bool current_killed_state = kill_switch_msg->is_killed;

    if(last_killed_state_ == false && current_killed_state == true)
    {
      ROS_INFO("Kill Switch CB: Killing");
      state_ = killing;
    }

		last_killed_state_ = current_killed_state;
	}

  // ######################################################################
  // Wait for the error between the base link and the desired pose is less
  // than a threshold. Throw an std::exception if the kill switch is pulled
  void waitForPose()
  {
    tf::Transform error_pose_;
    do
    {
      usleep(10000);
      if(state_ == killing) 
      {
        ROS_INFO("Wait For Pose: Killing");
        throw std::logic_error("Kill Switch Pulled");
      }
      tf_utils::fetchTfFrame(error_pose_, "/seabee3/base_link", "/seabee3/desired_pose");
    }
    while (error_pose_.getOrigin().length() > error_threshold_);
  }

  // ######################################################################
  void letsDoThis()
  {
    //////////////////////////////
    // Dive
    //////////////////////////////
    ROS_INFO("Diving...");
    double roll, pitch, yaw;
    current_pose_.getBasis().getEulerYPR( yaw, pitch, roll );
    {
      seabee3_common::SetDesiredPose set_desired_pose_;
      set_desired_pose_.request.ori.values.z = yaw;
      set_desired_pose_.request.ori.mask.z   = 1;
      set_desired_pose_.request.pos.values.z = -depth_;
      set_desired_pose_.request.pos.mask.z   = 1;
      set_desired_pose_cli_.call( set_desired_pose_.request, set_desired_pose_.response );
    }
    waitForPose();

    //////////////////////////////
    // Go Through Gate
    //////////////////////////////
    ROS_INFO("Cruising to gate...");
    {
      seabee3_common::SetDesiredPose set_desired_pose_;
      set_desired_pose_.request.ori.values.x = distance_to_gate_;
      set_desired_pose_.request.ori.mask.x   = 1;
      set_desired_pose_cli_.call( set_desired_pose_.request, set_desired_pose_.response );
    }
    waitForPose();

    //////////////////////////////
    // Begin search for buoy
    //////////////////////////////
    

  }

  // ######################################################################
	void spinOnce()
	{
    tf_utils::fetchTfFrame( current_pose_, "/landmark_map", "/seabee3/base_link" ); 

    if(state_ == waiting_for_start)
    {
      FSMthread_ = std::thread(std::bind(&CompetitionDemo::letsDoThis, this));
      state_ = running;
    }
    else if(state_ == killing)
    {
      ROS_INFO("Spin Once: Killing");
      try { FSMthread_.join(); } catch(...) {}
      state_ = waiting_for_start;
      ROS_INFO("Spin Once: Killed.. Waiting for start");
    }
    
    
  }

};


int main( int argc, char ** argv )
{
	ros::init( argc, argv, "competition_demo" );
	ros::NodeHandle nh( "~" );

	CompetitionDemo competition_demo( nh );
	competition_demo.spin();

	return 0;
}
