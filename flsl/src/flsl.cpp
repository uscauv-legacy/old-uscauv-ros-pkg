#include <ros/ros.h>
#include <tf/tf.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>

#include <fstream>
#include <vector>
#include <math.h>
#include <stdlib.h>

int main( int argc, char* argv[] )
{
	ros::init(argc, argv, "flsl");
	ros::NodeHandle n;
	n.param("map_topic", map_topic, "/landmark_map");
	
	
}
