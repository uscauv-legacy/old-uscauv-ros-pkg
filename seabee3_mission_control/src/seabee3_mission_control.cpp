#include <vector>
#include <math.h>
#include <stdlib.h>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <landmark_finder/FindLandmarks.h>
#include <landmark_map_server/FetchLandmarkMap.h>
#include <landmark_map/LandmarkMap.h>
#include <seabee3_driver/SetDesiredXYZ.h>
#include <seabee3_driver/SetDesiredRPY.h>
#include <seabee3_driver_base/KillSwitch.h>
#include <seabee3_driver_base/FiringDeviceAction.h>
#include <waypoint_controller/SetDesiredPose.h>

/* SeaBee3 Mission States */
#define STATE_INIT          0
#define STATE_DO_GATE       1
#define STATE_FIRST_BUOY    2
#define STATE_SECOND_BUOY   3
#define STATE_HEDGE         4
#define STATE_FIRST_BIN     5
#define STATE_SECOND_BIN    6
#define STATE_WINDOW        7
#define STATE_PINGER        8


// max speed sub can go
#define MAX_SPEED              75.0

#define BUOY_HIT_DISTANCE     1.0
#define BIN_CENTER_THRESHOLD  0.3
#define WINDOW_CENTER_THRESHOLD  0.3

// ######################################################################
// The various types of SensorVotes
enum SensorType { PATH, FIRST_BUOY, SECOND_BUOY, BIN, SENSOR_TYPE_SIZE = 4};

// A vote for what the pose of the sub should be according to
// one of our sensors (i.e. salient point or path found in bottom cam)
struct SensorVote
{
  //public:
  // Represents a pose that a SensorVote can set
  struct SensorPose
  {
    float val; // value of pose
    float weight; // weight of pose
    float decay; // how fast the weight should decay (0 = none)
  };
  enum SensorType type; // sensor type
  SensorPose heading; // sensor's vote for absolute heading
  SensorPose strafe; // sensor's vote for relative strafe
  SensorPose depth; // sensor's vote for relative depth
  bool init; // whether or not the SensorVote has a value set

  SensorVote()
  {
    type = PATH;
    heading.val = 0.0;
    heading.weight = 0.0;
    heading.decay = 0.0;
    strafe.val = 0.0;
    strafe.weight = 0.0;
    strafe.decay = 0.0;
    depth.val = 0.0;
    depth.weight = 0.0;
    depth.decay = 0.0;
    init = false;
  }

  SensorVote(SensorType t)
  {
    type = t;
    heading.val = 0.0;
    heading.weight = 0.0;
    heading.decay = 0.0;
    strafe.val = 0.0;
    strafe.weight = 0.0;
    strafe.decay = 0.0;
    depth.val = 0.0;
    depth.weight = 0.0;
    depth.decay = 0.0;
    init = false;
  }
};
// ######################################################################

// Stores the sub's current mission state
unsigned int mCurrentState;

// Command-line parameters
double mGateTime;
double mGateDepth;
double mHeadingCorrScale;
double mDepthCorrScale;
double mSpeedCorrScale;

// Stores the current state of the kill switch according to BeeStemI
char mKillSwitchState;

// Vector that stores all of the sub's SensorVotes
std::vector<SensorVote> mSensorVotes;

// The combined error of our current heading and depth from our desired heading and depth
float mPoseError;

// Stores our currently desired dive value. Used to store intermediate dive values when
// as we dive in 4 stages in the beginning.
int mDepthVal;
int mDepthError;

// The current dive state we are on (1-4)
char mDiveCount;

// Used to store the heading we want to use to go through the gate at the beginning of the mission.
int mHeadingVal;
int mHeadingError;

// Whether or not we should set a new speed in the control loop
bool mSpeedEnabled;

// The last time weights were decayed
ros::Time mLastDecayTime;

// Map Landmarks
int mFirstBuoyColor;
Landmark mFirstBuoy;
int mSecondBuoyColor;
Landmark mSecondBuoy;
Landmark mFirstBin;
int mFirstBinImage;
Landmark mSecondBin;
int mSecondBinImage;
int mWindowColor;
Landmark mWindow;
Landmark mPinger;


// Landmark Map
LandmarkMap mLandmarkMap;

// Publishers / Subscribers / Clients
ros::Subscriber kill_switch_sub;
ros::ServiceClient driver_depth;
ros::ServiceClient driver_rpy;
ros::ServiceClient dropper_one_srv;
ros::ServiceClient dropper_two_srv;
ros::ServiceClient shooter_srv;
ros::ServiceClient landmark_finder_srv;
ros::ServiceClient landmark_map_srv;
ros::ServiceClient waypoint_controller_srv;
ros::Publisher driver_speed;

void resetWeights()
{
  for(unsigned int i = 0; i < mSensorVotes.size(); i++)
    {
      mSensorVotes[i].heading.weight = 0.0;
      mSensorVotes[i].depth.weight = 0.0;
    }
}

void resetSensorVoteValues()
{
  for(unsigned int i = 0; i < mSensorVotes.size(); i++)
    {
      mSensorVotes[i].heading.val = 0.0;
      mSensorVotes[i].depth.val = 0.0;
    }
}

void decayWeights()
{
  for(unsigned int i = 0; i < mSensorVotes.size(); i++)
    {
      SensorVote sv = mSensorVotes[i];

      if(sv.heading.weight > 0.0)
	{
	  if(sv.heading.decay >= 0.0 && sv.heading.decay <= 1.0)
	    sv.heading.weight *= 1.0 - sv.heading.decay;
	}

      if(sv.depth.weight > 0.0)
	{
	  if(sv.depth.decay >= 0.0 && sv.depth.decay <= 1.0)
	    sv.depth.weight *= 1.0 - sv.depth.decay;
	}

      mSensorVotes[i] = sv;
    }
}

// ######################################################################
void set_depth(int depth)
{
  seabee3_driver::SetDesiredXYZ xyz_srv;
  xyz_srv.request.Mode.z = 1;
  xyz_srv.request.Mask.z = 1;
  xyz_srv.request.DesiredXYZ.z = depth;

  if (driver_depth.call(xyz_srv))
    {
      //ROS_INFO("Set Desired Depth: %d", depth_srv.response.CurrentDesiredDepth);
      mDepthError = xyz_srv.response.ErrorInXYZ.z;
    }
  else
    {
      ROS_ERROR("Failed to call service setDesiredXYZ");
    }
}

void set_speed(int speed)
{
  geometry_msgs::Twist cmd_vel;
  cmd_vel.linear.x = speed;
  cmd_vel.linear.y = 0;
  cmd_vel.linear.z = 0;
  
  cmd_vel.angular.x = 0;
  cmd_vel.angular.y = 0;
  cmd_vel.angular.z = 0;

  driver_speed.publish(cmd_vel);
}

void set_strafe(int strafe)
{
  geometry_msgs::Twist cmd_vel;
  cmd_vel.linear.x = 0;
  cmd_vel.linear.y = strafe;
  cmd_vel.linear.z = 0;
  
  cmd_vel.angular.x = 0;
  cmd_vel.angular.y = 0;
  cmd_vel.angular.z = 0;
  
  driver_speed.publish(cmd_vel);
}

void set_heading(int heading)
{
  seabee3_driver::SetDesiredRPY rpy_srv;
  rpy_srv.request.Mode.x = 0;
  rpy_srv.request.Mode.y = 0;
  rpy_srv.request.Mode.z = 1;
  rpy_srv.request.Mask.x = 0;
  rpy_srv.request.Mask.y = 0;
  rpy_srv.request.Mask.z = 1;
  rpy_srv.request.DesiredRPY.x = 0;
  rpy_srv.request.DesiredRPY.y = 0;
  rpy_srv.request.DesiredRPY.z = heading;

  if (driver_rpy.call(rpy_srv))
    {
      //      ROS_INFO("Set Desired RPY: %f", rpy_srv.response.CurrentDesiredRPY.z);
      mHeadingError = rpy_srv.response.ErrorInRPY.z;
    }
  else
    {
      ROS_ERROR("Failed to call service setDesiredRPY");
    }
}

void resetPID()
{
  set_depth(0);
  set_heading(0);
  set_speed(0);
}

void killSwitchCallback(const seabee3_driver_base::KillSwitchConstPtr & msg)
{
  mKillSwitchState = msg->Value;
}

void initLandmarkMap()
{
  landmark_map_server::FetchLandmarkMap fetch_map;
  fetch_map.request.Req = 1;

  landmark_map_srv.call(fetch_map);

  mLandmarkMap = LandmarkMap(fetch_map.response.Map);

  std::vector<Landmark> buoys = mLandmarkMap.fetchLandmarksByType(Landmark::LandmarkType::Buoy);

  for(unsigned int i = 0; i < buoys.size(); i++)
    {
      if(buoys[i].mColor == mFirstBuoyColor)
	mFirstBuoy = buoys[i];
      else if(buoys[i].mColor == mSecondBuoyColor)
	mSecondBuoy = buoys[i];
    }

  std::vector<Landmark> bins = mLandmarkMap.fetchLandmarksByType(Landmark::LandmarkType::Bin);

  for(unsigned int i = 0; i < bins.size(); i++)
    {
      if(bins[i].mColor == mFirstBinImage)
	mFirstBin = bins[i];
      else if(bins[i].mColor == mSecondBinImage)
	mSecondBin = bins[i];
    }

 std::vector<Landmark> windows = mLandmarkMap.fetchLandmarksByType(Landmark::LandmarkType::Window);

  for(unsigned int i = 0; i < windows.size(); i++)
    {
      if(windows[i].mColor == mWindowColor)
	mWindow = windows[i];
    }

  mPinger = (mLandmarkMap.fetchLandmarksByType(Landmark::LandmarkType::Pinger))[0];
}

void setWaypoint(geometry_msgs::Vector3 point)
{
  waypoint_controller::SetDesiredPose set_pose;
  set_pose.request.Pos.Mode.x = 1;
  set_pose.request.Pos.Mode.y = 1;
  set_pose.request.Pos.Mode.z = 1;
  
  set_pose.request.Pos.Mask.x = 1;
  set_pose.request.Pos.Mask.y = 1;
  set_pose.request.Pos.Mask.z = 1;
  
  set_pose.request.Pos.Values.x = point.x;
  set_pose.request.Pos.Values.y = point.y;
  set_pose.request.Pos.Values.z = point.z;

  set_pose.request.Ori.Mode.x = 0;
  set_pose.request.Ori.Mode.y = 0;
  set_pose.request.Ori.Mode.z = 0;
  
  set_pose.request.Ori.Mask.x = 0;
  set_pose.request.Ori.Mask.y = 0;
  set_pose.request.Ori.Mask.z = 0;
  
  set_pose.request.Ori.Values.x = 0;
  set_pose.request.Ori.Values.y = 0;
  set_pose.request.Ori.Values.z = 0;

  waypoint_controller_srv.call(set_pose);  
}

void setWaypoint(cv::Point3d point)
{
  waypoint_controller::SetDesiredPose set_pose;
  set_pose.request.Pos.Mode.x = 1;
  set_pose.request.Pos.Mode.y = 1;
  set_pose.request.Pos.Mode.z = 1;
  
  set_pose.request.Pos.Mask.x = 1;
  set_pose.request.Pos.Mask.y = 1;
  set_pose.request.Pos.Mask.z = 1;
  
  set_pose.request.Pos.Values.x = point.x;
  set_pose.request.Pos.Values.y = point.y;
  set_pose.request.Pos.Values.z = point.z;

  set_pose.request.Ori.Mode.x = 0;
  set_pose.request.Ori.Mode.y = 0;
  set_pose.request.Ori.Mode.z = 0;
  
  set_pose.request.Ori.Mask.x = 0;
  set_pose.request.Ori.Mask.y = 0;
  set_pose.request.Ori.Mask.z = 0;
  
  set_pose.request.Ori.Values.x = 0;
  set_pose.request.Ori.Values.y = 0;
  set_pose.request.Ori.Values.z = 0;

  waypoint_controller_srv.call(set_pose);  
}

// ######################################################################
void state_init()
{
  ROS_INFO("Doing initial dive");
  
  // Give diver 5 seconds to point sub at gate
  sleep(5);
}


// ######################################################################
void state_do_gate()
{
  ROS_INFO("Moving towards gate...");

  // Set speed to MAX_SPEED
  set_speed(MAX_SPEED);
  // Sleep for mGateFwdTime
  sleep(mGateTime);

  ROS_INFO("Finished going through gate...");

  // Start following saliency to go towards flare
  mCurrentState = STATE_FIRST_BUOY;
}

// ######################################################################
void state_first_buoy()
{
  ROS_INFO("Hitting First Buoy...");

  // Request a buoy position update
  landmark_finder::FindLandmarks find_buoy;
  find_buoy.request.Type = Landmark::LandmarkType::Buoy;
  find_buoy.request.Ids.push_back(mFirstBuoyColor);

  // if a buoy is found
  if(landmark_finder_srv.call(find_buoy))
    {      
      // check for state transition: i.e. buoy is close enough to be
      // considered a "hit" and update to next state
      if(find_buoy.response.Landmarks.LandmarkArray[0].Center.x <= BUOY_HIT_DISTANCE)
	{
	  mCurrentState = STATE_SECOND_BUOY;
	}
      // otherwise manuever towards visible buoy 
      else
	setWaypoint(find_buoy.response.Landmarks.LandmarkArray[0].Center);
    }
  // otherwise move towards estimated buoy waypoint on map
  else
    {
      setWaypoint(mFirstBuoy.mCenter);
    }      
}

// ######################################################################
void state_second_buoy()
{
  ROS_INFO("Hitting Second Buoy...");

  // Request a buoy position update
  landmark_finder::FindLandmarks find_buoy;
  find_buoy.request.Type = Landmark::LandmarkType::Buoy;
  find_buoy.request.Ids.push_back(mSecondBuoyColor);

  // if a buoy is found
  if(landmark_finder_srv.call(find_buoy))
    {
      // check for state transition: i.e. buoy is close enough to be
      // considered a "hit" and update to next state
      if(find_buoy.response.Landmarks.LandmarkArray[0].Center.x <= BUOY_HIT_DISTANCE)
	{
	  mCurrentState = STATE_HEDGE;
	}
      // otherwise move towards visible buoy
      else
	setWaypoint(find_buoy.response.Landmarks.LandmarkArray[0].Center);
    }
  // otherwise move towards estimated buoy waypoint on map
  else
    {
      setWaypoint(mSecondBuoy.mCenter);
    }   
}
// ######################################################################
void state_hedge()
{
  ROS_INFO("Maneuvering Hedge...");
  
  // Request a bin position update
  landmark_finder::FindLandmarks find_bin;
  find_bin.request.Type = Landmark::LandmarkType::Bin;
  find_bin.request.Ids.push_back(mFirstBinImage);
 
  // if a bin is found
  if(landmark_finder_srv.call(find_bin))
    {
      // if we are centered on bin, drop markers and move to next state
      if(find_bin.response.Landmarks.LandmarkArray[0].Center.x <= BIN_CENTER_THRESHOLD &&
	 find_bin.response.Landmarks.LandmarkArray[0].Center.y <= BIN_CENTER_THRESHOLD)
	{
	  // drop marker
	  seabee3_driver_base::FiringDeviceAction fire_dropper;
	  fire_dropper.request.Req = 1;
	  dropper_one_srv.call(fire_dropper);

	  // go to next state
	  mCurrentState = STATE_FIRST_BIN;
	} 
      // maneuver towards visible bin
      else
	setWaypoint(find_bin.response.Landmarks.LandmarkArray[0].Center);
    }
  else
    {
      setWaypoint(mFirstBin.mCenter);
    }
}


// ######################################################################
void state_first_bin()
{
  ROS_INFO("Dropping Markers into First Bin...");

  // Request a bin position update
  landmark_finder::FindLandmarks find_bin;
  find_bin.request.Type = Landmark::LandmarkType::Bin;
  find_bin.request.Ids.push_back(mFirstBinImage);
 
  // if a bin is found
  if(landmark_finder_srv.call(find_bin))
    {
      // if we are centered on bin, drop markers and move to next state
      if(find_bin.response.Landmarks.LandmarkArray[0].Center.x <= BIN_CENTER_THRESHOLD &&
	 find_bin.response.Landmarks.LandmarkArray[0].Center.y <= BIN_CENTER_THRESHOLD)
	{
	  // drop marker
	  seabee3_driver_base::FiringDeviceAction fire_dropper;
	  fire_dropper.request.Req = 1;
	  dropper_one_srv.call(fire_dropper);

	  // go to next state
	  mCurrentState = STATE_SECOND_BIN;
	} 
      // maneuver towards visible bin
      else
	setWaypoint(find_bin.response.Landmarks.LandmarkArray[0].Center);
    }
  else
    {
      setWaypoint(mFirstBin.mCenter);
    }
}

// ######################################################################
void state_second_bin()
{
  ROS_INFO("Dropping Markers into Second Bin...");

  // Request a bin position update
  landmark_finder::FindLandmarks find_bin;
  find_bin.request.Type = Landmark::LandmarkType::Bin;
  find_bin.request.Ids.push_back(mSecondBinImage);

  // if a bin is found
  if(landmark_finder_srv.call(find_bin))
    {
      // if we are centered on bin, drop markers and move to next state
      if(find_bin.response.Landmarks.LandmarkArray[0].Center.x <= BIN_CENTER_THRESHOLD &&
	 find_bin.response.Landmarks.LandmarkArray[0].Center.y <= BIN_CENTER_THRESHOLD)
	{
	  // drop marker
	  seabee3_driver_base::FiringDeviceAction fire_dropper;
	  fire_dropper.request.Req = 1;
	  dropper_two_srv.call(fire_dropper);

	  // go to next state
	  mCurrentState = STATE_WINDOW;
	} 
      // maneuver towards visible bin
      else
	setWaypoint(find_bin.response.Landmarks.LandmarkArray[0].Center);
    }
  else
    {
      setWaypoint(mSecondBin.mCenter);
    }
}

// ######################################################################
void state_window()
{
  ROS_INFO("Shooting torpedo through marker...");

  // Request a buoy position update
  landmark_finder::FindLandmarks find_window;
  find_window.request.Type = Landmark::LandmarkType::Window;

  // if a bin is found
  if(landmark_finder_srv.call(find_window))
    {
      // manuever towards window
      setWaypoint(find_window.response.Landmarks.LandmarkArray[0].Center);

      // if we are centered on window, shoot torpedo and move to next state
      if(find_window.response.Landmarks.LandmarkArray[0].Center.x <= WINDOW_CENTER_THRESHOLD &&
	 find_window.response.Landmarks.LandmarkArray[0].Center.z <= WINDOW_CENTER_THRESHOLD)
	{
	  // shoot torpedo
	  seabee3_driver_base::FiringDeviceAction fire_shooter;
	  fire_shooter.request.Req = 1;
	  shooter_srv.call(fire_shooter);

	  // move to next state
	  mCurrentState = STATE_PINGER;
	} 
    }
  else
    {
      setWaypoint(mSecondBuoy.mCenter);
    }

}

// ######################################################################
void state_pinger()
{
  ROS_INFO("Going to pinger...");

}
// ######################################################################

int main(int argc, char** argv)
{
  mCurrentState = STATE_INIT;
  mKillSwitchState = 1;
  mPoseError = -1.0;
  mDepthVal = -1;
  mDepthError = 0;
  mDiveCount = 0;
  mHeadingVal = -1;
  mHeadingError = 0;
  mSpeedEnabled = false;
  resetPID();

  ros::init(argc, argv, "seabee3_mission_control");
  ros::NodeHandle n("~");

  // initialize command-line parameters
  n.param("gate_time", mGateTime, 40.0);
  n.param("gate_depth", mGateDepth, 85.0);
  n.param("heading_corr_scale", mHeadingCorrScale, 125.0);
  n.param("depth_corr_scale", mDepthCorrScale, 50.0);
  n.param("speed_corr_scale", mSpeedCorrScale, 1.0);
  n.param("first_buoy_color", mFirstBuoyColor, 0);
  n.param("second_buoy_color", mSecondBuoyColor, 2);
  n.param("first_bin_image", mFirstBinImage, 0);
  n.param("second_bin_image", mSecondBinImage, 1);
  n.param("window_color", mWindowColor, 0);

  kill_switch_sub = n.subscribe("seabee3/KillSwitch", 100, killSwitchCallback);
	
  driver_depth = n.serviceClient<seabee3_driver::SetDesiredXYZ>("seabee3/setDesiredXYZ");
  driver_rpy = n.serviceClient<seabee3_driver::SetDesiredRPY>("seabee3/setDesiredRPY");
  driver_speed = n.advertise<geometry_msgs::Twist>("seabee3/cmd_vel", 1);
  dropper_one_srv = n.serviceClient<seabee3_driver_base::FiringDeviceAction>("seabee3/Dropper1Action");
  dropper_two_srv = n.serviceClient<seabee3_driver_base::FiringDeviceAction>("seabee3/Dropper2Action");
  shooter_srv = n.serviceClient<seabee3_driver_base::FiringDeviceAction>("seabee3/ShooterAction");
  landmark_finder_srv = n.serviceClient<landmark_finder::FindLandmarks>("landmark_finder/FindBuoys");
  landmark_map_srv = n.serviceClient<landmark_map_server::FetchLandmarkMap>("landmark_map_server/fetchLandmarkMap");
  waypoint_controller_srv = n.serviceClient<waypoint_controller::SetDesiredPose>("waypoint_controller/setDesiredPose");
  
  initLandmarkMap();

  while(ros::ok())
    {	  
      // If the kill switch is active, act based on the sub's current state.
      if(mKillSwitchState == 0)
	{
	  switch(mCurrentState)
	    {
	    case STATE_INIT:
	      state_init();
	      break;
	    case STATE_DO_GATE:
	      state_do_gate();
	      break;
	    case STATE_FIRST_BUOY:
	      state_first_buoy();
	      break;
	    case STATE_SECOND_BUOY:
	      state_second_buoy();
	      break;
	    case STATE_HEDGE:
	      state_hedge();
	      break;
	    case STATE_FIRST_BIN:
	      state_first_bin();
	      break;
	    case STATE_SECOND_BIN:
	      state_second_bin();
	      break;
	    case STATE_WINDOW:
	      state_window();
	      break;
	    case STATE_PINGER:
	      state_pinger();
	      break;
	    }
	}
      else
	{
	  ROS_INFO("Waiting for kill switch...\n");
	  resetPID();
	  
	  mDepthVal = -1;
	  mDepthError = 0;
	  mHeadingVal = -1;
	  mHeadingError = 0; 
	  mCurrentState = STATE_INIT;
	}

      ros::Rate(20).sleep();
    }
	
  return 0;
}
