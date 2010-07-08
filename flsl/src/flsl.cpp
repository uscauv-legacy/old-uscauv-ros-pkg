#include <ros/ros.h>
#include <tf/tf.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>

#include <vector>
#include <math.h>
#include <string>

std::string map_topic;

int main( int argc, char* argv[] )
{
	ros::init(argc, argv, "flsl");
	ros::NodeHandle n;
	n.param("map_topic", map_topic, std::string("/landmark_map") );
	
	
}
