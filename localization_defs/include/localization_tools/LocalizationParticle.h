#include <opencv/cv.h>

class LocalizationParticle
{
public:

	struct State
	{
		cv::Point3d mCenter;
		double mHeading;
		double mWeight;
		
		State(cv::Point3d center = cv::Point3d(0.0, 0.0, 0.0), double heading = 0.0, double weight = 0.0)
		{
			mCenter = center;
			mHeading = heading;
			mWeight = weight;
		}
	};
	
	LocalizationParticle()
	{
		//
	}
	
	LocalizationParticle(State state)
	{
		mState = state;
	}
	
	State mState;
};
