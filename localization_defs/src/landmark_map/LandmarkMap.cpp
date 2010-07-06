#include <landmark_map/LandmarkMap.h>

LandmarkMap::LandmarkMap( cv::Point2d dim )
{
	mDim = dim;
}

LandmarkMap::LandmarkMap(Landmark l)
{
	addLandmark(l);
}

LandmarkMap::LandmarkMap(std::vector<Landmark> l)
{
	for(unsigned int i = 0; i < l.size(); i ++)
	{
		addLandmark(l[i]);
	}
}

void LandmarkMap::addLandmark(const Landmark l)
{
	mLandmarks.push_back(l);
}

std::vector<Landmark> LandmarkMap::fetchLandmarksByType(const int type) const
{
	std::vector<Landmark> landmarks;
	for(unsigned int i = 0; i < mLandmarks.size(); i ++)
	{
		if(mLandmarks[i].mLandmarkType == type)
		{
			landmarks.push_back(mLandmarks[i]);
		}
	}
	return landmarks;
}

std::vector<visualization_msgs::Marker> LandmarkMap::createMarkerArray(const std::string & frame) const
{
	std::vector<visualization_msgs::Marker> markers;
	markers.resize( mLandmarks.size() );
	
	std::vector<int> typeCount;
	typeCount.resize(Landmark::LandmarkType::NumTypes);
	
	for(unsigned int i = 0; i < markers.size(); i ++)
	{
		markers[i] = mLandmarks[i].createMarker( frame, typeCount[mLandmarks[i].mLandmarkType] );
		typeCount[ mLandmarks[i].mLandmarkType ] ++;
	}
	
	return markers;
}

localization_defs::LandmarkMapMsg LandmarkMap::createMsg() const
{
	localization_defs::LandmarkMapMsg msg;
	for(int i = 0; i < mLandmarks.size(); i ++)
	{
		msg.Landmarks.push_back( mLandmarks[i].createMsg() );
	}
	
	return msg;
}
	
