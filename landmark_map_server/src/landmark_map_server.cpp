#include <ros/ros.h>
#include <yaml-cpp/yaml.h>
#include <opencv/cv.h>

#include <string>
#include <fstream>

#include <landmark_map/LandmarkMap.h>
#include <localization_defs/LandmarkMapMsg.h>
#include <visualization_msgs/MarkerArray.h>

std::string map_uri, map_frame;
std::vector<Landmark> mLandmarks;

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

int main( int argc, char* argv[] )
{
	ros::init(argc, argv, "landmark_map_server");
	ros::NodeHandle n("~");
	
	n.param("map_uri", map_uri, std::string("null") );
	n.param("map_frame", map_frame, std::string("/landmark_map") );
	
	ROS_INFO("Attempting to open map file at %s", map_uri.c_str());
	
	std::ifstream fin(map_uri.c_str());
	
	if(!fin.good())
	{
		ROS_ERROR("File not found.");
		return 1;
	}
	
	ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("landmarks", 0);
	ros::Publisher map_pub = n.advertise<localization_defs::LandmarkMapMsg>("landmark_map", 0);
	
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
	localization_defs::LandmarkMapMsg mapMsg = mMap.createMsg();
	
	while(ros::ok())
	{
		// get a visualization_msgs/Marker representation of each landmark and publish it
	
		for( unsigned int i = 0; i < markers.size(); i ++ )
		{
			marker_pub.publish(markers[i]);
		}
		
		map_pub.publish(mapMsg);
	
		ros::spinOnce();
		ros::Rate(2).sleep();
	}
	return 1;
}
