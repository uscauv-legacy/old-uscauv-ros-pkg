/*******************************************************************************
 *
 *      flsl
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

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <opencv/cv.h>

#include <vector>
#include <math.h>
#include <string>

#include <landmark_map/LandmarkMap.h>
#include <landmark_map_server/FetchLandmarkMap.h>
//#include <localization_tools/LandmarkSensor.h>

std::string map_frame;
//std::vector <LocalizationParticle> mParticles;
//cv::Point3d mEstimatedPosition;
tf::Transform mOdomCorrection;
tf::TransformBroadcaster * tb;

ros::ServiceClient fetch_landmark_map_srv;
landmark_map_server::FetchLandmarkMap fetchLandmarkMap;

void updateLocation()
{
	fetch_landmark_map_srv.call(fetchLandmarkMap);
	//mach up shit on map, update odom correction
	mOdomCorrection.setOrigin( tf::Vector3( 0.0, 0.0, 0.0 ) );
	mOdomCorrection.setRotation( tf::Quaternion( 0.0, 0.0, 0.0 ) );
	tb->sendTransform( tf::StampedTransform( mOdomCorrection, ros::Time::now(), "/landmark_map", "/seabee3/odom" ) );
}

/*void calculatePosition( std::vector & particles, cv::Point3d & mEstimatedPosition)
{
	for(int i = 0; i < particles.size(); i ++)
	{
		particles
	}
}*/

int main( int argc, char* argv[] )
{
	ros::init(argc, argv, "flsl");
	ros::NodeHandle n("~");
	
	n.param("map_frame", map_frame, std::string("map") );
	
	fetch_landmark_map_srv = n.serviceClient<landmark_map_server::FetchLandmarkMap>("/landmark_map_server/fetchLandmarkMap");
	
	tb = new tf::TransformBroadcaster;
	
	while( ros::ok() )
	{
		updateLocation();
		ros::spinOnce();
		ros::Rate(2).sleep();
	}
	return 1;
}
