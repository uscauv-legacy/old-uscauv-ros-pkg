#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <seabee3_driver/SetDesiredRPY.h>
#include <seabee3_driver/SetDesiredXYZ.h>
#include <landmark_map/LandmarkMap.h>
#include <localization_defs/LandmarkMapMsg.h>
#include <string>
#include <tf/transform_listener.h>
#include <control_toolbox/pid.h>
#include <localization_tools/Util.h>
#include <waypoint_controller/SetDesiredPose.h>
#include <waypoint_controller/FineTunePose.h>
#include <waypoint_controller/ReleasePose.h>
#include <waypoint_controller/CurrentState.h>
#include <opencv/cv.h>

std::string robot_frame, map_frame;

struct PIDConfig
{
	double p;
	double i;
	double d;
};

struct State
{
	const static int idle = -1;
	const static int fine_tune = 0;
	const static int set_xyz = 1;  //initial heading + depth
	const static int move_xyz = 2;
	const static int translate = 3; //minimize x+y error
	const static int set_rpy = 4;  //set final rpy
	const static int move_rpy = 5;
	const static int holding = 6; //hold final x+y
};

int mState;

control_toolbox::Pid * pid_X, * pid_Y;
tf::StampedTransform currentPose;
tf::TransformListener * tl;

geometry_msgs::Vector3 errorInPos, errorInOri;

//geometry_msgs::Vector3 desiredPos, desiredOri;

double maxErrorInPos, maxErrorInOri, strafeOnlyThreshold;

ros::ServiceClient set_desired_rpy_srv;
ros::ServiceClient set_desired_xyz_srv;

seabee3_driver::SetDesiredRPY setDesiredRPY;
seabee3_driver::SetDesiredXYZ setDesiredXYZ;

geometry_msgs::Vector3 waypointRPY, waypointXYZ, navGoalRPY;

ros::Time lastPidUpdateTime;

void operator -= (geometry_msgs::Vector3 & v1, geometry_msgs::Vector3 & v2)
{
	v1.x -= v2.x;
	v1.y -= v2.y;
	v1.z -= v2.z;
}

bool updatePID(geometry_msgs::Twist & cmd_vel)
{
	ROS_INFO("update PID");
	if(lastPidUpdateTime != ros::Time(-1))
	{
		ros::Duration dt = ros::Time::now() - lastPidUpdateTime;
		ROS_INFO("dt %f", dt.toSec());
		
		LocalizationUtil::capValue(errorInPos.x, 10.0);
		LocalizationUtil::capValue(errorInPos.y, 10.0);
		
		//errorInPos.setX(errorInPos.x);
		//errorInPos.setY(errorInPos.y);
		
		cmd_vel.linear.x = -pid_X->updatePid(errorInPos.x, dt);
		cmd_vel.linear.y = -pid_Y->updatePid(errorInPos.y, dt);
		
		LocalizationUtil::capValue(cmd_vel.linear.x, 0.75);
		LocalizationUtil::capValue(cmd_vel.linear.y, 0.75);
		
		setDesiredXYZ.request.Mask.z = 1;
		setDesiredXYZ.request.Mode.z = 1;
		setDesiredXYZ.request.DesiredXYZ.z = errorInPos.z;
	}
	
	lastPidUpdateTime = ros::Time::now();
	return true;
}

bool updateCmdVel(geometry_msgs::Twist & cmd_vel)
{
	//ros::Duration dt = ros::Time::now() - lastPidUpdateTime;
	
	if(mState == State::idle || mState == State::fine_tune)
		return false;
	
	if(mState >= State::set_xyz)
	{
		try
		{
			tl->waitForTransform(map_frame.c_str(), robot_frame.c_str(), ros::Time(0), ros::Duration(5.0));
			tl->lookupTransform(map_frame.c_str(), robot_frame.c_str(), ros::Time(0), currentPose);
			//listener.lookupTransform("/turtle2", "/turtle1", ros::Time(0), transform);
		}
		catch (tf::TransformException ex)
		{
			ROS_ERROR("%s",ex.what());
		}
		
		tf::Vector3 theTf = currentPose.getOrigin();
		cv::Point3d theOri;
		currentPose.getBasis().getEulerYPR(theOri.x, theOri.y, theOri.z);
		
		ROS_INFO("tf x %f y %f z %f r %f p %f y %f", theTf.x(), theTf.y(), theTf.z(), theOri.x, theOri.y, theOri.z);
		
		//tl->lookupTransform(map_frame.c_str(), robot_frame.c_str(), ros::Time(0), currentPose);
		
		errorInPos = waypointXYZ;
		errorInPos.x -= currentPose.getOrigin().x();
		errorInPos.y -= currentPose.getOrigin().y();
		errorInPos.z -= currentPose.getOrigin().z();
		
		ROS_INFO("error in pos x %f y %f z %f", errorInPos.x, errorInPos.y, errorInPos.z);
		
		errorInOri = navGoalRPY;
		geometry_msgs::Vector3 currentOri;
		currentPose.getBasis().getEulerYPR(currentOri.z, currentOri.y, currentOri.x);
		
		currentOri.z = LocalizationUtil::radToDeg( currentOri.z );
		currentOri.y = LocalizationUtil::radToDeg( currentOri.y );
		currentOri.x = LocalizationUtil::radToDeg( currentOri.x );
		
		LocalizationUtil::normalizeAngle( currentOri.z );
		LocalizationUtil::normalizeAngle( currentOri.y );
		LocalizationUtil::normalizeAngle( currentOri.z );
		
		errorInOri -= currentOri;
		
		ROS_INFO("error in ori x %f y %f z %f", errorInOri.x, errorInOri.y, errorInOri.z);
		
		if(mState == State::translate || mState == State::holding)
		{
			if(errorInPos.x + errorInPos.y + errorInPos.z < maxErrorInPos)
			{
				//mState = mState == State::translate ? State::set_rpy : mState;
				//cmd_vel.linear.x = cmd_vel.linear.y = cmd_vel.linear.z = cmd_vel.angular.x = cmd_vel.angular.y = cmd_vel.angular.z = 0.0;
				mState = mState == State::translate ? State::set_rpy : State::holding;
			}
			updatePID( cmd_vel );
			return true;
		}
	}
	if(mState == State::set_xyz)
	{
		set_desired_xyz_srv.call(setDesiredXYZ);
		
		//cv::Point2d waypointRelPos( desiredPos.x() - , desiredXYZ.y );
		
		//calculate the distance and angle from our current location to the desired waypoint
		cv::Point2d vectorToWaypoint = LocalizationUtil::vectorTo( cv::Point2d( 0, 0 ), cv::Point2d( errorInPos.x, errorInPos.y ) );
		
		ROS_INFO("vector to waypoint %f @ %f", vectorToWaypoint.x, vectorToWaypoint.y);
		
		if(vectorToWaypoint.y > strafeOnlyThreshold)
		{
			ROS_INFO("rotating to face the waypoint");
			//rotate to face waypoint, then drive forward to reach it
			seabee3_driver::SetDesiredRPY temp;
			temp.request.Mask.z = 1; //activate yaw
			temp.request.Mode.z = 1; //set this orientation relative to our current orientation
			temp.request.DesiredRPY.z = vectorToWaypoint.x; //set the desired yaw
			set_desired_rpy_srv.call( temp );
			
			navGoalRPY.x = 0;
			navGoalRPY.y = 0;
			navGoalRPY.z = vectorToWaypoint.y;
		}
		else
		{
			ROS_INFO("the waypoint is close enough that we can just strafe to it");
			navGoalRPY = waypointRPY;
			//set goal RPY, then strafe
			set_desired_rpy_srv.call(setDesiredRPY);
		}
		
		mState = State::move_xyz;
		return false;
	}
	else if(mState == State::move_xyz)
	{
		//mState = errorInPos.z < maxErrorInPos ? State::translate : mState;
		if(errorInPos.z < maxErrorInPos && errorInOri.z < maxErrorInOri )
		{
			ROS_INFO("errorInPos.z %f errorInOri.z %f", errorInPos.z, errorInOri.z);
			mState = State::translate;
		}
		//return mState == State::translate;
		return false;
	}
	else if(mState == State::set_rpy)
	{
		set_desired_rpy_srv.call(setDesiredRPY);
		mState = State::move_rpy;
	}
	else if(mState == State::move_rpy)
	{
		//mState = errorInOri.x + errorInOri.y + errorInOri.z <= maxErrorInOri ? State::holding : mState;
		if(errorInOri.x + errorInOri.y + errorInOri.z <= maxErrorInOri)
		{
			mState = State::holding;
		}
	}
	else if(mState == State::holding)
	{
		return true;
	}
	
	//lastPidUpdateTime = ros::Time::now();
	return false;
}

bool FineTunePoseCallback( waypoint_controller::FineTunePose::Request & req, waypoint_controller::FineTunePose::Response & resp)
{
	mState = State::fine_tune;
	errorInPos = req.Pos;
	return true;
}

bool SetDesiredPoseCallback( waypoint_controller::SetDesiredPose::Request & req, waypoint_controller::SetDesiredPose::Response & resp)
{
	waypointRPY.x = req.Ori.x;
	waypointRPY.y = req.Ori.y;
	waypointRPY.z = req.Ori.z;
	
	waypointXYZ.x = req.Pos.x;
	waypointXYZ.y = req.Pos.y;
	waypointXYZ.z = req.Pos.z;
	
	mState = State::set_xyz;
	
	return true;
}

bool ReleasePoseCallback( waypoint_controller::ReleasePose::Request & req, waypoint_controller::ReleasePose::Response & resp)
{
	mState = State::idle;
	return true;
}

int main( int argc, char * argv[] )
{
	ros::init(argc, argv, "waypoint_controller");
	ros::NodeHandle n("~");
	
	n.param("robot_frame", robot_frame, std::string("/base_link") );
	n.param("map_frame", map_frame, std::string("/landmark_map") );
	
	ros::Publisher cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/seabee3/cmd_vel", 1);
	ros::Publisher state_pub = n.advertise<waypoint_controller::CurrentState>("current_state", 1);
	
	set_desired_rpy_srv = n.serviceClient<seabee3_driver::SetDesiredRPY>("setRPY");
	set_desired_xyz_srv = n.serviceClient<seabee3_driver::SetDesiredXYZ>("setXYZ");
	
	ros::ServiceServer set_desired_pose_srv = n.advertiseService("setDesiredPose", SetDesiredPoseCallback);
	ros::ServiceServer fine_tune_pose_srv = n.advertiseService("fineTunePose", FineTunePoseCallback);
	ros::ServiceServer release_desired_pose_srv = n.advertiseService("releasePose", ReleasePoseCallback);
	
	tl = new tf::TransformListener;
	
	lastPidUpdateTime = ros::Time(-1);
	
	pid_X = new control_toolbox::Pid;
	pid_Y = new control_toolbox::Pid;
	
	PIDConfig pid_X_cfg, pid_Y_cfg;
	
	double pid_i_min, pid_i_max;
	
	n.param("max_error_in_pos", maxErrorInPos, 5.0);
	n.param("max_error_in_ori", maxErrorInOri, 0.25);
	n.param("strafe_only_threshold", strafeOnlyThreshold, 3.0);
	
	n.param("pid/X/p", pid_X_cfg.p, 0.8);
	n.param("pid/X/i", pid_X_cfg.i, 0.0);
	n.param("pid/X/d", pid_X_cfg.d, 0.0);
	
	n.param("pid/Y/p", pid_Y_cfg.p, 0.8);
	n.param("pid/Y/i", pid_Y_cfg.i, 0.0);
	n.param("pid/Y/d", pid_Y_cfg.d, 0.0);
	
	n.param("pid/i_max", pid_i_max, 1.0);
	n.param("pid/i_min", pid_i_min, -1.0);
	
	pid_X->initPid(pid_X_cfg.p, pid_X_cfg.i, pid_X_cfg.d, pid_i_max, pid_i_min);
	pid_X->reset();
	pid_Y->initPid(pid_Y_cfg.p, pid_Y_cfg.i, pid_Y_cfg.d, pid_i_max, pid_i_min);
	pid_Y->reset();
	
	geometry_msgs::Twist desiredVelocity;
	waypoint_controller::CurrentState currentState;
	
	mState = State::idle;
	
	while( ros::ok() )
	{
		if( updateCmdVel(desiredVelocity) )
		{
			cmd_vel_pub.publish( desiredVelocity );
		}
		else if(mState == State::fine_tune)
		{
			updatePID( desiredVelocity );
			cmd_vel_pub.publish( desiredVelocity );
		}
		
		currentState.State = mState;
		state_pub.publish( currentState );
		
		ros::spinOnce();
		ros::Rate(20).sleep();
	}
	
	return 0;
}
