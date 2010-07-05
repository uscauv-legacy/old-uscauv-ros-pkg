#include <ros/ros.h>
#include <yaml-cpp/yaml.h>

#include <string>

#include <landmark_map/landmark_map.h>

string map_uri;

int main( int argc, char* argv[] )
{
	ros::init(argc, argv, "landmark_map_server");
	ros::NodeHandle n;
	
	n.param("map_uri", map_uri, "");
	
	std::ifstream fin(map_uri);
	
	if(!fin.good())
	{
		ROS_WARN("File not found.");
		return 1;
	}
	
	YAML::Parser parser(fin);
	YAML::Node doc;
	parser.GetNextDocument(doc);
	mSegments.resize( doc.size() );
	
	for( unsigned int i = 0; i < doc.size(); i ++ )
	{
		doc[i] >> mLandmarks[i];
	}
	
	// for each landmark
	
	for( unsigned int i = 0; i < mLandmarks.size(); i ++ )
	{
		mSegments[i].drawSegment(map_msg.data);
	}
}
