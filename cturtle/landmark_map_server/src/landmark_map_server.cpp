/*******************************************************************************
 *
 *      landmark_map_server
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
#include <yaml-cpp/yaml.h>
#include <opencv/cv.h>

#include <string>
#include <fstream>

#include <landmark_map/LandmarkMap.h>
#include <localization_defs/LandmarkMapMsg.h>
#include <visualization_msgs/MarkerArray.h>
#include <landmark_map_server/FetchLandmarkMap.h>

std::string map_uri;
const std::string map_frame = "/landmark_map";
std::vector<Landmark> mLandmarks;
localization_defs::LandmarkMapMsg mapMsg;

void operator >> (const YAML::Node & node, cv::Point3d & p)
{
	node[0] >> p.x;
	node[1] >> p.y;
	node[2] >> p.z;
	ROS_INFO("yaml -> cv::Point3d( %f, %f, %f )", p.x, p.y, p.z);
}

void operator >> (const YAML::Node & node, cv::Point2d & p)
{
	node[0] >> p.x;
	node[1] >> p.y;
	ROS_INFO("yaml -> cv::Point2d( %f, %f )", p.x, p.y);
}

void operator >> (const YAML::Node & node, Landmark & l)
{
	int landmarkType;
	cv::Point3d center;
	double ori;
	
	node["type"] >> landmarkType;
	node["center"] >> center;
	node["ori"] >> ori;
	
	Landmark * result = new Landmark;
	
	if(landmarkType == Landmark::LandmarkType::Buoy) //used 'if' instead of 'switch' so that variables could be defiend inside selection logic
	{
		int color;
		node["color"] >> color;
		LandmarkTypes::Buoy buoy ( center, ori, color );
		
		result = &buoy;
	}
	else if(landmarkType == Landmark::LandmarkType::Pinger)
	{
		int id;
		node["id"] >> id;
		LandmarkTypes::Pinger pinger (center, ori, id);
		
		result = &pinger;
	}
	else if(landmarkType == Landmark::LandmarkType::Pipe)
	{
		LandmarkTypes::Pipe pipe (center, ori);
		result = &pipe;
	}
	else if(landmarkType == Landmark::LandmarkType::Bin)
	{
		int id;
		node["id"] >> id;
		LandmarkTypes::Bin bin (center, ori, id);
		result = &bin;
	}
	else if(landmarkType == Landmark::LandmarkType::Window)
	{
		int color;
		node["color"] >> color;
		LandmarkTypes::Window window (center, ori, color);
		result = &window;
	}
	else if(landmarkType == Landmark::LandmarkType::Waypoint)
	{
		int id;
		node["id"] >> id;
		LandmarkTypes::Waypoint waypoint (center, ori, id);
		result = &waypoint;
	}
	else
	{
		ROS_WARN("Landmark ID %d is not defined. Landmark not added to map.", landmarkType);
	}
	
	l = *result;
	ROS_INFO("Landmark ID %d", result->mLandmarkType);
	ROS_INFO("Landmark ID %d", l.mLandmarkType);
}

#define USAGE "Usage: \n" \
              "  landmark_map_saver [-f <mapname>] [ROS remapping args]"

bool FetchLandmarkMapCallback(landmark_map_server::FetchLandmarkMap::Request & req, landmark_map_server::FetchLandmarkMap::Response & resp)
{
	resp.Map = mapMsg;
	return true;
}

int main( int argc, char* argv[] )
{
	ros::init(argc, argv, "landmark_map_server");
	ros::NodeHandle n("~");
	
	n.param("map_uri", map_uri, std::string("null") );
	//n.param("map_frame", map_frame, std::string("/landmark_map") );
	
	ROS_INFO("Attempting to open map file at %s", map_uri.c_str());
	
	std::ifstream fin(map_uri.c_str());
	
	if(!fin.good())
	{
		ROS_ERROR("File not found.");
		return 1;
	}
	
	ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("landmarks", 5);
	ros::ServiceServer map_server = n.advertiseService("fetchLandmarkMap", FetchLandmarkMapCallback);
	
	YAML::Parser parser(fin);
	YAML::Node doc;
	ROS_INFO("instantiated the yaml parser");
	
	parser.GetNextDocument(doc);
	ROS_INFO("did 'GetNextDocument'");
	
	LandmarkMap mMap;
	
	doc["dim"] >> mMap.mDim;
	ROS_INFO("set dim");
	
	const YAML::Node & landmarks = doc["landmarks"];
	ROS_INFO("got top node for landmarks");
	
	//populate the list of landmarks
	
	for( unsigned int i = 0; i < landmarks.size(); i ++ )
	{
		Landmark temp;
		landmarks[i] >> temp;
		mMap.addLandmark(temp);
		ROS_INFO("Landmark type: %d", temp.mLandmarkType);
	}
	
	ROS_INFO("populated list of landmarks");
	
	std::vector<visualization_msgs::Marker> markers = mMap.createMarkerArray( map_frame );
	mapMsg = mMap.createMsg();
	
	while(ros::ok())
	{
		// get a visualization_msgs/Marker representation of each landmark and publish it
	
		for( unsigned int i = 0; i < markers.size(); i ++ )
		{
			marker_pub.publish(markers[i]);
		}
		
		//map_pub.publish(mapMsg);
	
		ros::spinOnce();
		ros::Rate(2).sleep();
	}
	return 1;
}
