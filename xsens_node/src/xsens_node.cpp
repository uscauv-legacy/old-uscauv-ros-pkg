#include "xsens_lib/xsens_driver.h"

#include "xsens_node/IMUData.h"
#include "xsens_node/CalibrateRPY.h"

#include <ros/ros.h>
#include <tf/tf.h>
#include <iostream>
#include <queue>

std::queue<tf::Vector3> * ori_data_cache;
tf::Vector3 * ori_comp;
tf::Vector3 * ori_comp_total;
XSensDriver * mImuDriver;
float sampleTime;

void operator += (XSensDriver::Vector3 & v1, tf::Vector3 & v2)
{
	v1.x += v2.getX();
	v1.y += v2.getY();
	v1.z += v2.getZ();
}

void operator >> (XSensDriver::Vector3 & v1, geometry_msgs::Vector3 & v2)
{
	v2.x = v1.x;
	v2.y = v1.y;
	v2.z = v1.z;
}

void operator >> (XSensDriver::Vector3 & v1, tf::Vector3 & v2)
{
	v2.setX(v1.x);
	v2.setY(v1.y);
	v2.setZ(v1.z);
}

void operator >> (tf::Vector3 & v1, geometry_msgs::Vector3 & v2)
{
	v2.x = v1.getX();
	v2.y = v1.getY();
	v2.z = v1.getZ();
}

void updateIMUData()
{
	if( !mImuDriver->updateData() )
	{
		ROS_WARN("Failed to update data during this cycle...");
		//fprintf(stdout, "-");
	}
	else
	{
		//fprintf(stdout, ".");
		//std::cout << std::flush;
		tf::Vector3 temp;
		mImuDriver->ori >> temp;
		ori_data_cache->push(temp);
		
		while(ori_data_cache->size() > 2)
		{
			ori_data_cache->pop();
		}
	}
}

void runRPYCalibration(int n)
{
	for(int i = 0; i < n && ros::ok(); i ++)
	{
		updateIMUData();
		tf::Vector3 diff (ori_data_cache->front() - ori_data_cache->back());
		*ori_comp += diff;
		ros::spinOnce();
		ros::Rate(10).sleep();
		//ROS_INFO("sample %d: x %f y %f z %f", i, diff.getX(), diff.getY(), diff.getZ());
	}
	
	sampleTime = (float)n * 0.1f;
	
	*ori_comp /= (float)(-n);
}

bool CalibrateRPYCallback (xsens_node::CalibrateRPY::Request &req, xsens_node::CalibrateRPY::Response &res)
{
	int numSamples = req.ZeroReq;
	
	runRPYCalibration(numSamples);
	
	*ori_comp >> res.Result;
	
	return true;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "xsens_node");
	ros::NodeHandle n;
	ros::Publisher imu_pub = n.advertise<xsens_node::IMUData>("IMUData", 100);
	ros::ServiceServer CalibrateRPY_srv = n.advertiseService("CalibrateRPY", CalibrateRPYCallback);
	
	sampleTime = 1.0;
	ori_comp = new tf::Vector3(0, 0, 0);
	ori_comp_total = new tf::Vector3(0, 0, 0);
	ori_data_cache = new std::queue<tf::Vector3>;
	
	int usbIndex;
	n.param("usbIndex", usbIndex, 1);
	
	mImuDriver = new XSensDriver((unsigned int)usbIndex);
	if( !mImuDriver->initMe() )
	{
		ROS_WARN("Failed to connect to IMU. Exiting...");
		return 1;
	}
	
	runRPYCalibration(10);
	
	while(ros::ok())
	{
		xsens_node::IMUData msg;
		
		updateIMUData();
		
		mImuDriver->accel >> msg.accel;
		mImuDriver->gyro >> msg.gyro;
		mImuDriver->mag >> msg.mag;
		
		*ori_comp_total += (*ori_comp * ((1.0f / 110.0f) / sampleTime));
		
		//ROS_INFO("Offset x %f y %f z %f", ori_comp_total->getX(), ori_comp_total->getY(), ori_comp_total->getZ());
		
		mImuDriver->ori += *ori_comp_total;
		mImuDriver->ori >> msg.ori;
		
		imu_pub.publish(msg);
		ros::spinOnce();
		ros::Rate(110).sleep();
	}
	
	delete mImuDriver;
	delete ori_comp;
	delete ori_comp_total;
	delete ori_data_cache;
	
	return 0;
}
