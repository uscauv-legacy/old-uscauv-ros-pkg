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
    ros::ServiceClient set_enabled_landmarks_cli_;

    ros::Subscriber landmarks_sub_;

    tf::Transform current_pose_;

    enum FSMState_t {waiting_for_start, starting, running, killing};
    FSMState_t state_;
    std::thread FSMthread_;

    bool landmark_found_;

    bool tracking_landmark_;

    std::mutex mtx_;

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

    kill_switch_sub_ = nh_local_.subscribe( "/seabee3/kill_switch", 2, &CompetitionDemo::killSwitchCB, this );
    set_desired_pose_cli_ = nh_local_.serviceClient<seabee3_common::SetDesiredPose> ( "/seabee3/set_desired_pose" );
    reset_pose_cli_ = nh_local_.serviceClient<std_srvs::Empty> ( "/seabee3/reset_pose" );

    landmarks_sub_ = nh_local_.subscribe( "/landmark_finder/landmarks", 3, &CompetitionDemo::landmarksCB, this );
    //set_enabled_landmarks_cli_ = nh_local_.serviceClient<landmark_finder::SetEnabledLandmarks>(
    //    "/landmark_finder/set_enabled_landmarks");
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
          std::lock_guard<std::mutex> lock(mtx);
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

      if(state_ == running && last_killed_state_ == false && current_killed_state == true)
      {
        ROS_INFO("Kill Switch CB: Killing");
        state_ = killing;
      }
      else if(state_ == waiting_for_start && last_killed_state_ == true && current_killed_state == false)
      {
        ROS_INFO("Kill Switch CB: Starting");
        state_ = starting;
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
      ROS_INFO("Let's do this!");

      //////////////////////////////
      // Dive
      //////////////////////////////
      ROS_INFO("Diving...");
      {
        double roll, pitch, yaw;
        {
          std::lock_guard<std::mutex> lock(mtx_);
          current_pose_.getBasis().getEulerYPR( yaw, pitch, roll );
        }
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
        set_desired_pose_.request.pos.values.x = distance_to_gate_;
        set_desired_pose_.request.pos.mask.x   = 1;
        set_desired_pose_cli_.call( set_desired_pose_.request, set_desired_pose_.response );
      }
      waitForPose();

      //////////////////////////////
      // Begin search for buoy
      //////////////////////////////
      ROS_INFO("Searching For Buoys...");
      {
        // TODO: Set enabled landmark here!!
        landmark_found_ = false;
        while(!landmark_found_)
        {
          double roll, pitch, yaw;
          {
            std::lock_guard<std::mutex> lock(mtx_);
            current_pose_.getBasis().getEulerYPR( yaw, pitch, roll );
          }
          seabee3_common::SetDesiredPose set_desired_pose_;
          set_desired_pose_.request.ori.values.z = yaw + M_PI/16;
          set_desired_pose_.request.ori.mask.z   = 1;
          set_desired_pose_cli_.call( set_desired_pose_.request, set_desired_pose_.response );
          waitForPose();
        }
      }

      ROS_INFO("Homing In On Buoy");
      {
        // Head towards landmark until within error tolerance:
        tracking_landmark_ = true;
        waitForPose();
        tracking_landmark_ = false;
      }

      ROS_INFO("Competition Finished.");
    }

    // ######################################################################
    void spinOnce()
    {
      {
        std::lock_guard<std::mutex> lock(mtx_);
        tf_utils::fetchTfFrame( current_pose_, "/landmark_map", "/seabee3/base_link" ); 
      }

      if(state_ == starting)
      {
        ROS_INFO("Spin Once: Starting Run");
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
