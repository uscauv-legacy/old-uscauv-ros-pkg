#include <base_node/base_node.h>
#include <geometry_msgs/Twist.h>
#include <common_utils/math.h>
#include <common_utils/tf.h>

// msgs
#include <seabee3_driver_base/KillSwitch.h>
#include <localization_defs/LandmarkArray.h>

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
    ros::ServiceClient reset_physics_cli_;
    ros::ServiceClient set_enabled_landmarks_cli_;

    ros::Subscriber landmarks_sub_;

    tf::Transform current_pose_;
    std::mutex mtx_;

    enum FSMState_t {waiting_for_start=0, starting=1, running=2, killing=3};
    FSMState_t state_;

    std::thread FSM_thread_;

    bool landmark_found_;

    bool tracking_landmark_;
    tf::TransformListener tf_listener_;

    bool do_buoy_;

    ////////////////////////////////
    // Run Parameters
    ////////////////////////////////
    double error_threshold_;
    double depth_;
    double distance_to_gate_;

  public:
    // ######################################################################
    CompetitionDemo( ros::NodeHandle & nh ) : BaseNode<> ( nh )
  {
    state_ = waiting_for_start;

    nh_local_.param( "error_threshold", error_threshold_, 0.1 );
    nh_local_.param( "depth", depth_, 1.7 );
    nh_local_.param( "distance_to_gate", distance_to_gate_, 10.0 );
    nh_local_.param( "do_buoy", do_buoy_, true );

	printf( "Do buoy: %d\n", do_buoy_ );

    kill_switch_sub_ = nh_local_.subscribe( "/seabee3/kill_switch", 2, &CompetitionDemo::killSwitchCB, this );
    set_desired_pose_cli_ = nh_local_.serviceClient<seabee3_common::SetDesiredPose> ( "/seabee3/set_desired_pose" );
    reset_pose_cli_ = nh_local_.serviceClient<std_srvs::Empty> ( "/seabee3/reset_pose" );
    reset_physics_cli_ = nh_local_.serviceClient<std_srvs::Empty> ( "/seabee3_physics/reset_pose" );

    landmarks_sub_ = nh_local_.subscribe( "/landmark_finder/landmarks", 3, &CompetitionDemo::landmarksCB, this );

    FSM_thread_ = std::thread(std::bind(&CompetitionDemo::FSMmethod, this));
  }

    // ######################################################################
    void landmarksCB( const localization_defs::LandmarkArray::ConstPtr & landmark_msg )
    {
      if(landmark_msg->landmarks.size() == 0) return;

      if(tracking_landmark_)
      {
        double x = landmark_msg->landmarks[0].pose.linear.x;
        double y = landmark_msg->landmarks[0].pose.linear.y;

        tf::Transform cp;
        {
          ROS_INFO("Locking LM");
          std::lock_guard<std::mutex> lock(mtx_);
          ROS_INFO("Locked LM");
          cp = current_pose_;
        }

        seabee3_common::SetDesiredPose set_desired_pose_;
        set_desired_pose_.request.pos.values.x = x;
        set_desired_pose_.request.pos.mask.x   = 1;
        set_desired_pose_.request.pos.values.y = y;
        set_desired_pose_.request.pos.mask.y   = 1;
        set_desired_pose_cli_.call( set_desired_pose_.request, set_desired_pose_.response );
      }
    }

    // ######################################################################
    void killSwitchCB( const seabee3_driver_base::KillSwitch::ConstPtr & kill_switch_msg )
    {
      static bool last_killed_state_ = kill_switch_msg->is_killed;
      bool current_killed_state = kill_switch_msg->is_killed;

      ROS_INFO("killSwitchCB: last_killed_state_:%d current_killed_state_:%d state_:%d", last_killed_state_, current_killed_state, state_);

      if(last_killed_state_ == false && current_killed_state == true)
      {
        ROS_INFO("Kill Switch CB: Killing");
        std_srvs::Empty empty_request;
        reset_pose_cli_.call( empty_request );
        reset_physics_cli_.call( empty_request );
        state_ = killing;
      }
      else if(last_killed_state_ == true && current_killed_state == false)
      {
        ROS_INFO("Kill Switch CB: Starting");
        state_ = starting;
      }
      last_killed_state_ = current_killed_state;
    }

	void fetchTfFrame( tf::Transform & transform, const std::string & from, const std::string & to )
	{
    ROS_INFO("Locking Fetch");
    std::lock_guard<std::mutex> lock(mtx_);
		tf_utils::fetchTfFrame( transform, from, to, ros::Time( 0 ), 0.1, &tf_listener_ );
    ROS_INFO("Locked Fetch");
	}
	
    // ######################################################################
    // Wait for the error between the base link and the desired pose is less
    // than a threshold. Returns false if the killswitch was pulled
    bool waitForPose(std::string posename = "")
    {
      sleep(1);
      ROS_INFO("Waiting For Pose...[%s]", posename.c_str());
      tf::Transform error_pose_;
      do
      {
        usleep(10000);
        if(state_ == killing) 
        { ROS_INFO("Wait For Pose: Killing"); return false; }
        fetchTfFrame(error_pose_, "/seabee3/base_link", "/seabee3/desired_pose");

        if(!ros::ok()) exit(0);

      }
      while (error_pose_.getOrigin().length() > error_threshold_);
      return true;
    }

    void FSMmethod()
    {
      ROS_INFO("Let's do this!");
      std_srvs::Empty empty_request;
      reset_pose_cli_.call( empty_request );
      reset_physics_cli_.call( empty_request );

      //////////////////////////////
      // Dive
      //////////////////////////////
      ROS_INFO("Diving...");
      {
        seabee3_common::SetDesiredPose set_desired_pose_;
        set_desired_pose_.request.pos.values.z = -depth_;
        set_desired_pose_.request.pos.mask.z   = 1;
        set_desired_pose_cli_.call( set_desired_pose_.request, set_desired_pose_.response );
      }
      if(!waitForPose("dive")) return;
      ROS_INFO("...Done Diving");

      //////////////////////////////
      // Go Through Gate
      //////////////////////////////
      ROS_INFO("Cruising to gate...");
      {
        tf::Transform gate_tf;
        fetchTfFrame( gate_tf, "/landmark_map", "/gate" ); 
        tf::Transform rel_gate_tf;
        fetchTfFrame( rel_gate_tf, "/seabee3/base_link", "/gate" ); 
        double yaw, pitch, roll;
        rel_gate_tf.getBasis().getEulerYPR( yaw, pitch, roll );

        seabee3_common::SetDesiredPose set_desired_pose_;
        set_desired_pose_.request.pos.values.x = gate_tf.getOrigin().x();
        set_desired_pose_.request.pos.mask.x   = 1;
        set_desired_pose_.request.pos.values.y = gate_tf.getOrigin().y();
        set_desired_pose_.request.pos.mask.y   = 1;
        set_desired_pose_.request.pos.values.z = gate_tf.getOrigin().z();
        set_desired_pose_.request.pos.mask.z   = 1;
        set_desired_pose_.request.ori.values.z = yaw; 
        set_desired_pose_cli_.call( set_desired_pose_.request, set_desired_pose_.response );
      }
      if(!waitForPose("gate")) return;
      ROS_INFO("...Done Gate");

      if(do_buoy_)
      {
        //////////////////////////////
        // Begin search for buoy
        //////////////////////////////
        ROS_INFO("Searching For Buoys...");
        {
          landmark_found_ = false;
          while(!landmark_found_)
          {
            double roll, pitch, yaw;

            {
              ROS_INFO("Locking_FSM");
              std::lock_guard<std::mutex> lock(mtx_);
              ROS_INFO("Locked_FSM");
              current_pose_.getBasis().getEulerYPR( yaw, pitch, roll );
            }

            seabee3_common::SetDesiredPose set_desired_pose_;
            set_desired_pose_.request.ori.values.z = yaw + M_PI/16;
            set_desired_pose_.request.ori.mask.z   = 1;
            set_desired_pose_cli_.call( set_desired_pose_.request, set_desired_pose_.response );
            if(!waitForPose("buoy search")) return;
          }
        }
        ROS_INFO("...Found Buoys");

        //////////////////////////////
        // Home in on the buoy
        //////////////////////////////
        ROS_INFO("Homing In On Buoy");
        {
          // Head towards landmark until within error tolerance:
          tracking_landmark_ = true;
          if(!waitForPose("buoy home")) return;
          tracking_landmark_ = false;
        }
        ROS_INFO("...Hit Buoys");
      }


      //////////////////////////////
      // Go to the hedge
      //////////////////////////////
      ROS_INFO("Going for the hedge");
      {
        seabee3_common::SetDesiredPose set_desired_pose_;
        tf::Transform hedge_tf;
        fetchTfFrame( hedge_tf, "/landmark_map", "/hedge" ); 
        tf::Transform rel_hedge_tf;
        fetchTfFrame( rel_hedge_tf, "/seabee3/base_link", "/hedge" ); 
        double yaw, pitch, roll;
        rel_hedge_tf.getBasis().getEulerYPR( yaw, pitch, roll );

        set_desired_pose_.request.pos.values.x = hedge_tf.getOrigin().x();
        set_desired_pose_.request.pos.mask.x   = 1;
        set_desired_pose_.request.pos.values.y = hedge_tf.getOrigin().y();
        set_desired_pose_.request.pos.mask.y   = 1;
        set_desired_pose_.request.pos.values.z = hedge_tf.getOrigin().z();
        set_desired_pose_.request.pos.mask.z   = 1;
        set_desired_pose_.request.ori.values.z = yaw; 
        set_desired_pose_cli_.call( set_desired_pose_.request, set_desired_pose_.response );
      }
      if(!waitForPose("hedge")) return;
      ROS_INFO("...Went through the hedge");

      //////////////////////////////
      // Go to the octagon
      //////////////////////////////
      ROS_INFO("Going for the octagon");
      {
        seabee3_common::SetDesiredPose set_desired_pose_;
        tf::Transform octagon_tf;
        fetchTfFrame( octagon_tf, "/landmark_map", "/octagon" ); 
        tf::Transform rel_octagon_tf;
        fetchTfFrame( rel_octagon_tf, "/seabee3/base_link", "/octagon" ); 
        double yaw, pitch, roll;
        rel_octagon_tf.getBasis().getEulerYPR( yaw, pitch, roll );

        set_desired_pose_.request.pos.values.x = octagon_tf.getOrigin().x();
        set_desired_pose_.request.pos.mask.x   = 1;
        set_desired_pose_.request.pos.values.y = octagon_tf.getOrigin().y();
        set_desired_pose_.request.pos.mask.y   = 1;
        set_desired_pose_.request.pos.values.z = octagon_tf.getOrigin().z();
        set_desired_pose_.request.pos.mask.z   = 1;
        set_desired_pose_.request.ori.values.z = yaw; 
        set_desired_pose_cli_.call( set_desired_pose_.request, set_desired_pose_.response );
      }
      if(!waitForPose("octagon")) return;
      ROS_INFO("...Went through the octagon");

      //////////////////////////////
      // Surface
      //////////////////////////////
      ROS_INFO("Surfacing");
      {
        seabee3_common::SetDesiredPose set_desired_pose_;

        set_desired_pose_.request.pos.values.z = -.5;
        set_desired_pose_.request.pos.mask.z   = 1;
        set_desired_pose_cli_.call( set_desired_pose_.request, set_desired_pose_.response );
      }
      if(!waitForPose("surface")) return;
      ROS_INFO("Surfaced");

      ROS_INFO("Competition Finished.");
    }

    // ######################################################################
    void letsDoThis()
    {
    }

    // ######################################################################
    void spinOnce()
    {
      if(state_ == starting)
      {
        ROS_INFO("Spin Once: Starting Run");
        state_ = running;
        FSM_thread_ = std::thread(std::bind(&CompetitionDemo::FSMmethod, this));
        ROS_INFO("Spin Once: Finished Run");
      }
      else if(state_ == killing)
      {
        ROS_INFO("Spin Once: Joining to kill");
        if(FSM_thread_.joinable()) { FSM_thread_.join(); }
        ROS_INFO("Spin Once: Joined");
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
