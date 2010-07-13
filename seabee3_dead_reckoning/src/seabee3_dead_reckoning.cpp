/*
Code taken from: http://www.ros.org/wiki/navigation/Tutorials/RobotSetup/Odom
*/


#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <seabee3_driver_base/MotorCntl.h>
#include <seabee3_driver_base/Pressure.h>
#include <tf/transform_broadcaster.h>
#include <xsens_node/IMUData.h>


//This is terrible, and needs to be put somewhere more logical and global
#define FWD_RIGHT_THRUSTER 	   3
#define FWD_LEFT_THRUSTER 	   1
#define DEPTH_RIGHT_THRUSTER 	 4
#define DEPTH_LEFT_THRUSTER 	 2
#define STRAFE_FRONT_THRUSTER	 0
#define STRAFE_BACK_THRUSTER 	 5
#define SHOOTER 	             6
#define DROPPER_STAGE1 	       7
#define DROPPER_STAGE2 	       8


///######################################################################
/// SpeedInterpolator
///   A class which contains a few stored mappings from motor speeds to
///   the times it took the sub to traverse some distance at those speeds
///
///   To make this work, you need to edit:
///     * itsTimedDistance in the constructor's initializer list
///     * itsTimingMap in the constructor body
///######################################################################
class SpeedInterpolator
{
	public:
	  SpeedInterpolator() :
			//Its timed distance should be the number of meters that the sub moved when you timed it.
		  itsTimedDistance(4.0) 
		{
			//This map needs to be filled out with timing data.
			//
			//  * The key (in the square[] brackets) should be the motor power for a run
			//
			//  * The value (after the =) should be the number of seconds it took the sub to move
			//    itsTimedDistance meters
			//  
			//  Make sure that you have values for both -100 and +100 in the map
			//
			itsTimingMap[-100] = 12.4;
			itsTimingMap[-50]  = 30.9;
			itsTimingMap[-10]  = 60.8;
			itsTimingMap[10]   = 45.9;
			itsTimingMap[50]   = 20.4;
			itsTimingMap[75]   = 10.4;
			itsTimingMap[100]  = 6.1;
		}

		//Returns the translational speed for a given motor power (the average
		//between the two forward thrusters)
		float getSpeed(int pwr)
		{
			if(pwr == 0)
				return 0.0;

			//Find the interpolated time it would take the seabee to traverse
			//"itsTimedDistance" at "pwr" speed.
			timingMap_t::iterator lower_it = itsTimingMap.lower_bound(pwr);
			timingMap_t::iterator upper_it = itsTimingMap.upper_bound(pwr);
			float interp_time;
			if(lower_it->first == pwr)
			{
				//If our lower iterator is an entry for the current power, then 
				//we don't need to interpolate
				interp_time = lower_it->second;
			}
			else
			{
				//Make sure lower it points to the entry that is _less_than_ the given power
				--lower_it;

				float lower_pwr = lower_it->first;
				float upper_pwr = upper_it->first;

				float lower_time = lower_it->second;
				float upper_time = upper_it->second;

				/*
						http://en.wikipedia.org/wiki/Linear_interpolation
						
						y = y_0 + (x - x_0)* (y_1 - y_0)/(x_1 - x_0)
						
						y-axis = time to travel itsTimedDistance meters
						x-axis = motor power
				*/
				interp_time = lower_time + (pwr - lower_pwr) * 
					(upper_time - lower_time)/(upper_pwr - lower_pwr);
			}

			// Speed = distance / time
			float interp_speed = itsTimedDistance / interp_time;

			//Make sure to recorrect for the sign of avgMotorPwr
			return interp_speed; 
		}

	private:
		//A mapping from motor speeds to the time it takes to travel "itsTimedDistance" meters
		typedef std::map<int, float> timingMap_t;
		timingMap_t itsTimingMap;

		//The number of meters the sub traveled during the timing runs
		float itsTimedDistance;
};
SpeedInterpolator theSpeedInterpolator;

//######################################################################

ros::Publisher odom_pub;

//TODO: Get this from the pressure sensor
double depth = 0.0;

//######################################################################
geometry_msgs::Vector3 ori;
void imuCallback(const xsens_node::IMUDataConstPtr & msg)
{
	ori = msg->ori;
}

//######################################################################
void motorCntlCallback(const seabee3_driver_base::MotorCntlConstPtr & msg)
{
	//Create some persistent variables for this callback
	static tf::TransformBroadcaster odom_broadcaster;
	static double x            =  0.0;
	static double y            =  0.0;
	static double z            =  0.0;
	static double th           =  0.0;
	static double last_depth   =  0.0;
	static double last_th      =  0.0;
	static ros::Time last_time =  ros::Time::now();
	ros::Time current_time     =  ros::Time::now();

	//Get the orientation from the imu 
	th = ori.z * M_PI/180.0;

	//Get the average forward speed
	int right_thrust_speed = msg->motors[FWD_RIGHT_THRUSTER];
	int left_thrust_speed = msg->motors[FWD_LEFT_THRUSTER];
	double trans_speed = theSpeedInterpolator.getSpeed( (right_thrust_speed + left_thrust_speed)/2.0 );

	//Find the amount of elapsed time
	double dt = (current_time - last_time).toSec();

	//Compute our predicted velocity from the motor commands
	double vx  = sin(th) * trans_speed;
	double vy  = cos(th) * trans_speed;
	double vth = (th - last_th) / dt;
	double vz  = (depth - last_depth)/dt;

	//compute the change in planar position from the speed and the elapsed time
	double delta_x = vx * dt;
	double delta_y = vy * dt;

	//Integrate!
	x  += delta_x;
	y  += delta_y;
	z  = depth;

	//First, publish the transform over tf
	//geometry_msgs::Quaternion odom_quat = tf::Quaternion(ori.z, ori.y, ori.x);
	tf::Quaternion quat(ori.z, ori.y, ori.x);
	geometry_msgs::Quaternion odom_quat;
	tf::quaternionTFToMsg(quat, odom_quat);
	geometry_msgs::TransformStamped odom_trans;
	odom_trans.header.stamp = current_time;
	odom_trans.header.frame_id = "odom";//<----------- Not sure what this is
	odom_trans.child_frame_id = "seabee_link"; //<---- Not sure what this is
	odom_trans.transform.translation.x = x;
	odom_trans.transform.translation.y = y;
	odom_trans.transform.translation.z = z;
	odom_trans.transform.rotation = odom_quat;
	odom_broadcaster.sendTransform(odom_trans);

	//Next, publish the odometry message over ROS
	nav_msgs::Odometry odom;
	odom.header.stamp = current_time;
	odom.header.frame_id = "odom";//<----------- Not sure what this is
	odom.pose.pose.position.x  = x;
	odom.pose.pose.position.y  = y;
	odom.pose.pose.position.z  = z;
	odom.pose.pose.orientation = odom_quat;
	odom.child_frame_id = "seabee_link";//<----- Not sure what this is
	odom.twist.twist.linear.x  = vx;
	odom.twist.twist.linear.y  = vy;
	odom.twist.twist.linear.z  = vz;
	odom.twist.twist.angular.z = vth;
	odom_pub.publish(odom);

	//Update our last_* variables
	last_time  = current_time;
	last_depth = depth;
	last_th = th;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "seabee3_dead_reckoning");
	ros::NodeHandle n("n");

	ros::Subscriber motor_cntl_sub = n.subscribe
		("/seabee3/motor_cntl", 1, motorCntlCallback);
	motor_cntl_sub = motor_cntl_sub;
	

	ros::Subscriber imu_sub = n.subscribe
		("/seabee3/data_raw", 1, imuCallback);
	imu_sub = imu_sub;

	odom_pub = n.advertise<nav_msgs::Odometry>("/seabee3/odom_prim", 50); 

	ros::spin();

	return 0;
}
