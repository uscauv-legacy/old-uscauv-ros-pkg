#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <seabee3_driver/SetDesiredRPY.h>
#include <seabee3_driver/SetDesiredXYZ.h>
#include <landmark_map/LandmarkMap.h>
#include <localization_defs/LandmarkMapMsg.h>
#include <string>
#include <tf/transform_listener.h>
#include <control_toolbox/pid.h>
#include <localization_tools/Util.h>
#include <waypoint_controller/SetDesiredPose.h>

std::string robot_frame, map_frame;

struct PIDConfig
{
	double p;
	double i;
	double d;
};

struct State
{
	const static int idle = 0;
	const static int set_xyz = 1;
	const static int move_xyz = 2;
	const static int moving = 3;
	const static int set_rpy = 4;
	const static int move_rpy = 5;
	const static int holding = 6;
};

int mState;

control_toolbox::Pid * pid_X, * pid_Y;
tf::StampedTransform currentPose;
tf::TransformListener * tl;
tf::Vector3 errorInPos;
tf::Vector3 errorInOri;
tf::Vector3 desiredPos;
tf::Vector3 desiredOri;

double maxErrorInPos, maxErrorInOri;

ros::ServiceClient set_desired_rpy_srv;
ros::ServiceClient set_desired_xyz_srv;

seabee3_driver::SetDesiredRPY setDesiredRPY;
seabee3_driver::SetDesiredXYZ setDesiredXYZ;
ros::Time lastPidUpdateTime;

geometry_msgs::Twist updateCmdVel()
{
	geometry_msgs::Twist cmd_vel;
	
	if(lastPidUpdateTime != ros::Time(-1))
	{
		ros::Duration dt = ros::Time::now() - lastPidUpdateTime;
		
		if(mState == State::idle)
			return cmd_vel;
		
		if(mState >= State::moving)
		{
			tl->waitForTransform(map_frame.c_str(), robot_frame.c_str(), ros::Time(0), ros::Duration(5.0));
			tl->lookupTransform(map_frame.c_str(), robot_frame.c_str(), ros::Time(0), currentPose);
		
			errorInPos = currentPose.getOrigin();
			errorInPos -= desiredPos;
			
			mState = errorInPos.x() + errorInPos.y() < maxErrorInPos ? State::set_rpy : mState;
			
			double cappedX = errorInPos.x(), cappedY = errorInPos.y();
			
			LocalizationUtil::capValue(cappedX, 20.0);
			LocalizationUtil::capValue(cappedY, 20.0);
			
			errorInPos.setX(cappedX);
			errorInPos.setY(cappedY);
			
			cmd_vel.linear.x = pid_X->updatePid(errorInPos.x(), dt);
			cmd_vel.linear.y = pid_Y->updatePid(errorInPos.y(), dt);
		}
		if(mState == State::set_xyz)
		{
			set_desired_xyz_srv.call(setDesiredXYZ);
		}
		else if(mState == State::move_xyz)
		{
			mState = errorInPos.z() < maxErrorInPos? State::moving : mState;
		}
		else if(mState == State::set_rpy)
		{
			set_desired_rpy_srv.call(setDesiredRPY);
			mState = State::move_rpy;
		}
		else if(mState == State::move_rpy)
		{
			mState = errorInOri.x() + errorInOri.y() + errorInOri.z() <= maxErrorInOri ? State::holding : mState;
		}
		else if(mState == State::holding)
		{
			//nothing to do; just keep maintaining position
		}
	}
	
	lastPidUpdateTime = ros::Time::now();
	return cmd_vel;
}

bool SetDesiredPoseCallback( waypoint_controller::SetDesiredPose::Request & req, waypoint_controller::SetDesiredPose::Response & resp)
{
	/*if(req.Pos.Mask.x == 1.0)
		desiredPos.setX(req.Pos.Values.x);
	if(req.Pos.Mask.y == 1.0)
		desiredPos.setY(req.Pos.Values.y);
	if(req.Pos.Mask.z == 1.0)
		desiredPos.setZ(req.Pos.Values.z);*/
		
	setDesiredXYZ.request.Mode = req.Pos.Mode;
	setDesiredXYZ.request.Mask = req.Pos.Mask;
	setDesiredXYZ.request.DesiredXYZ = req.Pos.Values;
	
	/*if(req.Ori.Mask.x == 1.0)
		desiredOri.setX(req.Ori.Values.x);
	if(req.Ori.Mask.y == 1.0)
		desiredOri.setY(req.Ori.Values.y);
	if(req.Ori.Mask.z == 1.0)
		desiredOri.setZ(req.Ori.Values.z);*/
	
	setDesiredRPY.request.Mode = req.Ori.Mode;
	setDesiredRPY.request.Mask = req.Ori.Mask;
	setDesiredRPY.request.DesiredRPY = req.Ori.Values;
	
	mState = State::moving;
		
	return true;
}

int main( int argc, char * argv[] )
{
	ros::init(argc, argv, "waypoint_controller");
	ros::NodeHandle n("~");
	
	n.param("robot_frame", robot_frame, std::string("/base_link") );
	n.param("map_frame", map_frame, std::string("/landmark_map") );
	
	ros::Publisher cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
	
	set_desired_rpy_srv = n.serviceClient<seabee3_driver::SetDesiredRPY>("setRPY");
	set_desired_xyz_srv = n.serviceClient<seabee3_driver::SetDesiredXYZ>("setXYZ");
	
	ros::ServiceServer set_desired_pose_srv = n.advertiseService("setDesiredPose", SetDesiredPoseCallback);
	
	tl = new tf::TransformListener;
	
	lastPidUpdateTime = ros::Time(-1);
	
	pid_X = new control_toolbox::Pid;
	pid_Y = new control_toolbox::Pid;
	
	PIDConfig pid_X_cfg, pid_Y_cfg;
	
	double pid_i_min, pid_i_max;
	
	n.param("max_error_in_pos", maxErrorInPos, 0.5);
	n.param("max_error_in_ori", maxErrorInOri, 0.5);
	
	n.param("pid/X/p", pid_X_cfg.p, 2.5);
	n.param("pid/X/i", pid_X_cfg.i, 0.05);
	n.param("pid/X/d", pid_X_cfg.d, 0.2);
	
	n.param("pid/Y/p", pid_Y_cfg.p, 2.5);
	n.param("pid/Y/i", pid_Y_cfg.i, 0.05);
	n.param("pid/Y/d", pid_Y_cfg.d, 0.2);
	
	n.param("pid/i_max", pid_i_max, 1.0);
	n.param("pid/i_min", pid_i_min, -1.0);
	
	pid_X->initPid(pid_X_cfg.p, pid_X_cfg.i, pid_X_cfg.d, pid_i_max, pid_i_min);
	pid_X->reset();
	pid_Y->initPid(pid_Y_cfg.p, pid_Y_cfg.i, pid_Y_cfg.d, pid_i_max, pid_i_min);
	pid_Y->reset();
	
	while( ros::ok() )
	{	
		cmd_vel_pub.publish( updateCmdVel() );
		
		ros::spinOnce();
		ros::Rate(20).sleep();
	}
	
	return 0;
}
