#include <landmark_map/LandmarkMap.h>

LandmarkMap::LandmarkMap()
{

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

void LandmarkMap::addLandmark(Landmark l)
{
	mLandmarks.resize(mLandmarks.size() + 1);
	mLandmarks[mLandmarks.size() - 1] = l;
}

std::vector<Landmark> LandmarkMap::fetchLandmarksById(const int id) const
{
	std::vector<Landmark> result;
	for(int i = 0; i < mLandmarks.size(); i ++)
	{
		if(mLandmarks[i].mLandmarkId == id)
		{
			result.push_back(mLandmarks[i]);
		}
	}
	return result;
}
