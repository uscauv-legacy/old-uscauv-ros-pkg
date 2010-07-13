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
#include <xsens_node/CalibrateRPY.h>
#include <xsens_node/SetRPYOffset.h>

#include <ros/ros.h>
#include <tf/tf.h>
#include <iostream>
#include <queue>

std::queue<tf::Vector3> * ori_data_cache;
tf::Vector3 * ori_comp;
tf::Vector3 * ori_offset;
tf::Vector3 * drift_comp;
tf::Vector3 * drift_comp_total;
//tf::Vector3 * rpy_zero_total;
XSensDriver * mImuDriver;
double * sampleTime;
const static unsigned int IMU_DATA_CACHE_SIZE = 2;

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

bool SetRPYOffsetCallback (xsens_node::SetRPYOffset::Request &req, xsens_node::SetRPYOffset::Response &res)
{
	//Mode = 1 : set change in rpy offset; else set absolute rpy offset
	if(req.Offset.Mask.x != 0.0)
		ori_offset->setX( req.Offset.Values.x + ( req.Offset.Mode.x == 1.0 ? ori_offset->x() : 0.0 ) );
	if(req.Offset.Mask.y != 0.0)
		ori_offset->setY( req.Offset.Values.y + ( req.Offset.Mode.y == 1.0 ? ori_offset->y() : 0.0 ) );
	if(req.Offset.Mask.z != 0.0)
		ori_offset->setZ( req.Offset.Values.z + ( req.Offset.Mode.z == 1.0 ? ori_offset->z() : 0.0 ) );
	
	*ori_offset >> res.Result;
	
	return true;
}

bool CalibrateRPYOriCallback (xsens_node::CalibrateRPY::Request &req, xsens_node::CalibrateRPY::Response &res)
{
	int numSamples = req.NumSamples;
	
	runRPYOriCalibration(numSamples);
	
	*ori_comp >> res.Result;
	
	return true;
}

bool CalibrateRPYDriftCallback (xsens_node::CalibrateRPY::Request &req, xsens_node::CalibrateRPY::Response &res)
{
	int numSamples = req.NumSamples;
	
	runRPYDriftCalibration(numSamples);
	
	*drift_comp >> res.Result;
	
	return true;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "xsens_node");
	ros::NodeHandle n;
	ros::Publisher imu_pub_calib = n.advertise<xsens_node::IMUData>("data_calibrated", 1);
	ros::Publisher imu_pub_raw = n.advertise<xsens_node::IMUData>("data_raw", 1);
	ros::ServiceServer CalibrateRPYDrift_srv = n.advertiseService("CalibrateRPYDrift", CalibrateRPYDriftCallback);
	ros::ServiceServer CalibrateRPYOri_srv = n.advertiseService("CalibrateRPYOri", CalibrateRPYOriCallback);
	ros::ServiceServer SetRPYOffset_srv = n.advertiseService("SetRPYOffset", SetRPYOffsetCallback);
	
	sampleTime = new double(1.0);
	ori_comp = new tf::Vector3(0, 0, 0);
	ori_offset = new tf::Vector3(0, 0, 0);
	drift_comp = new tf::Vector3(0, 0, 0);
	drift_comp_total = new tf::Vector3(0, 0, 0);
	
	ori_data_cache = new std::queue<tf::Vector3>;
	
	mImuDriver = new XSensDriver(0);
	if( !mImuDriver->initMe() )
	{
		ROS_WARN("Failed to connect to IMU. Exiting...");
		return 1;
	}
	
	runRPYOriCalibration(10);
	//runRPYDriftCalibration(10);
	
	while(ros::ok())
	{
		xsens_node::IMUData msg_calib;
		xsens_node::IMUData msg_raw;
		
		updateIMUData();
		
		mImuDriver->accel >> msg_raw.accel;
		mImuDriver->gyro >> msg_raw.gyro;
		mImuDriver->mag >> msg_raw.mag;
		mImuDriver->ori >> msg_raw.ori;
		
		mImuDriver->accel >> msg_calib.accel;
		mImuDriver->gyro >> msg_calib.gyro;
		mImuDriver->mag >> msg_calib.mag;
		
		*drift_comp_total += *drift_comp;// * *sampleTime;
		
		//ROS_INFO("Offset x %f y %f z %f", ori_comp_total->getX(), ori_comp_total->getY(), ori_comp_total->getZ());
		
		mImuDriver->ori += *drift_comp_total;
		
		tf::Vector3 temp;
		mImuDriver->ori >> temp;
		temp += *ori_comp + *ori_offset;
		
		temp >> msg_calib.ori;
		
		//mImuDriver->ori >> msg.ori;
		
		imu_pub_calib.publish(msg_calib);
		imu_pub_raw.publish(msg_raw);
		ros::spinOnce();
		ros::Rate(110).sleep();
	}
	
	delete mImuDriver;
	delete ori_comp;
	delete ori_offset;
	delete drift_comp;
	delete drift_comp_total;
	delete ori_data_cache;
	
	return 0;
}
