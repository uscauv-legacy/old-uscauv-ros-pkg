/*******************************************************************************
 *
 *      xsens_node
 * 
 *      Copyright (c) 2010, Edward T. Kaszubski (ekaszubski@gmail.com)
 *      All rights reserved.
 *
 *      Redistribution and use in source and binary forms, with or without
 *      modification, are permitted provided that the following conditions are
 *      met:
 *      
 *      * Redistributions of source code must retain the above copyright
 *        notice, this list of conditions and the following disclaimer.
 *      * Redistributions in binary form must reproduce the above
 *        copyright notice, this list of conditions and the following disclaimer
 *        in the documentation and/or other materials provided with the
 *        distribution.
 *      * Neither the name of the USC Underwater Robotics Team nor the names of its
 *        contributors may be used to endorse or promote products derived from
 *        this software without specific prior written permission.
 *      
 *      THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *      "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *      LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *      A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *      OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *      SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *      LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *      DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *      THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *      (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *      OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *******************************************************************************/

#include <xsens/xsens_driver.h>

#include <xsens_node/IMUData.h>
#include <xsens_node/CalibrateRPYOri.h>
#include <xsens_node/CalibrateRPYDrift.h>

#include <ros/ros.h>
#include <tf/tf.h>
#include <iostream>
#include <queue>

std::queue<tf::Vector3> * ori_data_cache;
tf::Vector3 * ori_comp;
tf::Vector3 * drift_comp;
tf::Vector3 * drift_comp_total;
//tf::Vector3 * rpy_zero_total;
XSensDriver * mImuDriver;
double * sampleTime;
const static int IMU_DATA_CACHE_SIZE = 2;

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
		
		while(ori_data_cache->size() >= IMU_DATA_CACHE_SIZE)
		{
			ori_data_cache->pop();
		}
		
		ori_data_cache->push(temp);
	}
}

void runRPYOriCalibration(int n = 10)
{
	*ori_comp *= 0.0; //reset the vector to <0, 0, 0>
	for(int i = 0; i < n && ros::ok(); i ++)
	{
		updateIMUData();
		*ori_comp += ori_data_cache->front();
		ros::spinOnce();
		ros::Rate(110).sleep();
		//ROS_INFO("sample %d: x %f y %f z %f", i, diff.getX(), diff.getY(), diff.getZ());
	}
	
	*ori_comp /= (double)(-n);
}

void runRPYDriftCalibration(int n = 10)
{
	*drift_comp *= 0.0; //reset the vector to <0, 0, 0>
	updateIMUData();
	*drift_comp = ori_data_cache->front();
	for(int i = 0; i < n && ros::ok(); i ++)
	{
		updateIMUData();
		ros::spinOnce();
		ros::Rate(110).sleep();
		//ROS_INFO("sample %d: x %f y %f z %f", i, diff.getX(), diff.getY(), diff.getZ());
	}
	*drift_comp -= ori_data_cache->front();
	
	//*sampleTime = (double)n / 110.0;
	
	*drift_comp /= (double)(n); //avg drift per cycle
}

bool CalibrateRPYOriCallback (xsens_node::CalibrateRPYOri::Request &req, xsens_node::CalibrateRPYOri::Response &res)
{
	int numSamples = req.ZeroReq;
	
	runRPYOriCalibration(numSamples);
	
	*ori_comp >> res.Result;
	
	return true;
}

bool CalibrateRPYDriftCallback (xsens_node::CalibrateRPYDrift::Request &req, xsens_node::CalibrateRPYDrift::Response &res)
{
	int numSamples = req.ZeroReq;
	
	runRPYDriftCalibration(numSamples);
	
	*drift_comp >> res.Result;
	
	return true;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "xsens_node");
	ros::NodeHandle n;
	ros::Publisher imu_pub = n.advertise<xsens_node::IMUData>("/xsens/IMUData", 1);
	ros::ServiceServer CalibrateRPYDrift_srv = n.advertiseService("/xsens/CalibrateRPYDrift", CalibrateRPYDriftCallback);
	ros::ServiceServer CalibrateRPYOri_srv = n.advertiseService("/xsens/CalibrateRPYOri", CalibrateRPYOriCallback);
	
	sampleTime = new double(1.0);
	ori_comp = new tf::Vector3(0, 0, 0);
	drift_comp = new tf::Vector3(0, 0, 0);
	drift_comp_total = new tf::Vector3(0, 0, 0);
	
	ori_data_cache = new std::queue<tf::Vector3>;
	
	int usbIndex;
	n.param("/xsens/usbIndex", usbIndex, 2);
	
	mImuDriver = new XSensDriver((unsigned int)usbIndex);
	if( !mImuDriver->initMe() )
	{
		ROS_WARN("Failed to connect to IMU. Exiting...");
		return 1;
	}
	
	runRPYOriCalibration(10);
	//runRPYDriftCalibration(10);
	
	while(ros::ok())
	{
		xsens_node::IMUData msg;
		
		updateIMUData();
		
		mImuDriver->accel >> msg.accel;
		mImuDriver->gyro >> msg.gyro;
		mImuDriver->mag >> msg.mag;
		
		*drift_comp_total += *drift_comp;// * *sampleTime;
		
		//ROS_INFO("Offset x %f y %f z %f", ori_comp_total->getX(), ori_comp_total->getY(), ori_comp_total->getZ());
		
		mImuDriver->ori += *drift_comp_total;
		
		tf::Vector3 temp;
		mImuDriver->ori >> temp;
		temp += *ori_comp;
		
		temp >> msg.ori;
		
		//mImuDriver->ori >> msg.ori;
		
		imu_pub.publish(msg);
		ros::spinOnce();
		ros::Rate(110).sleep();
	}
	
	delete mImuDriver;
	delete ori_comp;
	delete drift_comp;
	delete drift_comp_total;
	delete ori_data_cache;
	
	return 0;
}
