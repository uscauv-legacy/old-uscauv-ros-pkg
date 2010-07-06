#include <vector>
#include <landmark_map/Landmark.h>
#include <localization_defs/LandmarkMapMsg.h>
#include <opencv/cv.h>

class LandmarkMap
{
public:
	LandmarkMap( cv::Point2d dim = cv::Point2d(0.0, 0.0) );
	LandmarkMap(Landmark l);
	LandmarkMap(std::vector<Landmark> l);

	void addLandmark(const Landmark l);
	std::vector<Landmark> fetchLandmarksByType(const int type) const;
	std::vector<visualization_msgs::Marker> createMarkerArray(const std::string & frame) const;
	localization_defs::LandmarkMapMsg createMsg() const;

	std::vector<Landmark> mLandmarks;
	cv::Point2d mDim;
};
