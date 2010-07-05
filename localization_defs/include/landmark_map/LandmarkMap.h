#include <vector>
#include <landmark_map/Landmark.h>

class LandmarkMap
{
public:
	LandmarkMap();
	LandmarkMap(Landmark l);
	LandmarkMap(std::vector<Landmark> l);

	void addLandmark(Landmark l);
	std::vector<Landmark> fetchLandmarksById(const int id) const;

	std::vector<Landmark> mLandmarks;
};
