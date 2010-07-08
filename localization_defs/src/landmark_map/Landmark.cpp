#include <landmark_map/Landmark.h>
#include <tf/tf.h>
#include <sstream>
#include <localization_tools/Util.h>

Landmark::Landmark()
{
	mLandmarkType = Landmark::LandmarkType::None;
}

Landmark::Landmark(cv::Point3d center = cv::Point3d(0.0, 0.0, 0.0), double orientation = 0.0, cv::Point3d dim = cv::Point3d(1.0, 1.0, 1.0), int shapeType = visualization_msgs::Marker::ARROW)
{
	mLandmarkType = Landmark::LandmarkType::None;
	mCenter = center;
	mOrientation = orientation;
	mDim = dim;
	mShapeType = shapeType;
	mColor = Landmark::ColorIds::black;
	mId = -1;
}

visualization_msgs::Marker Landmark::createMarker(const std::string & frame, const int id, const std::string & ns_ext) const
{
	visualization_msgs::Marker marker;
	
	marker.header.frame_id = frame.c_str();
	marker.header.stamp = ros::Time::now();
	
	std::stringstream ss;
	
	switch (mLandmarkType)
	{
		case LandmarkType::None: ss << "landmark"; break;
		case LandmarkType::Buoy: ss << "buoy"; break;
		case LandmarkType::Pinger: ss << "pinger"; break;
		case LandmarkType::Pipe: ss << "pipe"; break;
	}
	
	ss << ns_ext << id;
	
	marker.ns = ss.str();
	marker.id = 0;
	marker.type = mShapeType;
	marker.action = visualization_msgs::Marker::ADD;
	
	marker.pose.position.x = mCenter.x;
	marker.pose.position.y = mCenter.y;
	marker.pose.position.z = mCenter.z;
	
	tf::Quaternion fromEuler ( LocalizationUtil::degToRad( mOrientation ) , 0, 0);
	
	marker.pose.orientation.x = fromEuler.x();
	marker.pose.orientation.y = fromEuler.y();
	marker.pose.orientation.z = fromEuler.z();
	marker.pose.orientation.w = fromEuler.w();
	
	marker.scale.x = mDim.x;
	marker.scale.y = mDim.y;
	marker.scale.z = mDim.z;
	
	const cv::Vec3b & color = Landmark::ColorDefs::getColor( mColor );
	
	marker.color.r = ((double)color[0] / 255.0);
	marker.color.g = ((double)color[1] / 255.0);
	marker.color.b = ((double)color[2] / 255.0);
	marker.color.a = 1.0;
	
	marker.lifetime = ros::Duration();
	
	return marker;
}

localization_defs::LandmarkMsg Landmark::createMsg() const
{
	localization_defs::LandmarkMsg msg;
	
	msg.Center.x = mCenter.x;
	msg.Center.y = mCenter.y;
	msg.Center.z = mCenter.z;
	
	msg.Ori = mOrientation;
	
	msg.Color = mColor;
	
	msg.Type = mLandmarkType;
	
	msg.Id = mId;
	
	return msg;
}

LandmarkTypes::Buoy::Buoy(cv::Point3d center, double orientation, int color) : 
Landmark(center, orientation, cv::Point3d(0.3048, 0.3048, 0.3048), visualization_msgs::Marker::SPHERE)
{
	mColor = color;
	mLandmarkType = Landmark::LandmarkType::Buoy;
}

/*visualization_msgs::Marker LandmarkTypes::Buoy::createMarker(std::string ns, std::string frame)
{
	const visualization_msgs::Marker & theMarker = Landmark::createMarker(ns, frame);
	
	theMarker.type = visualization_msgs::Marker::SPHERE;
	
	return theMarker;
}*/

LandmarkTypes::Pinger::Pinger(cv::Point3d center, double orientation, int id) : 
Landmark(center, orientation, cv::Point3d(0.0762, 0.0762, 0.1524), visualization_msgs::Marker::CYLINDER)
{
	mId = id;
	mColor = Landmark::ColorIds::green;
	mLandmarkType = Landmark::LandmarkType::Pinger;
}

/*visualization_msgs::Marker LandmarkTypes::Pinger::createMarker(std::string ns, std::string frame)
{
	const visualization_msgs::Marker & theMarker = Landmark::createMarker(ns, frame);
	
	theMarker.type = visualization_msgs::Marker::CUBE;
	
	return theMarker;
}*/

LandmarkTypes::Pipe::Pipe(cv::Point3d center, double orientation) : 
Landmark(center, orientation, cv::Point3d(1.2192, 0.6096, 0.0254), visualization_msgs::Marker::CUBE)
{
	mColor = Landmark::ColorIds::orange;
	mLandmarkType = Landmark::LandmarkType::Pipe;
}

/*visualization_msgs::Marker LandmarkTypes::Pipe::createMarker(std::string ns, std::string frame)
{
	const visualization_msgs::Marker & theMarker = Landmark::createMarker(ns, frame);
	
	theMarker.type = visualization_msgs::Marker::CUBE;
	
	return theMarker;
}*/
