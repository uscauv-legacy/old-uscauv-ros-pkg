#include <opencv/cv.h>

class Landmark
{
public:

	struct ShapeType
	{
		const static int line = 	0;
		const static int cube = 	1;
		const static int sphere = 	2;
		const static int cylinder = 3;
	};
	
	struct LandmarkId
	{
		const static int Buoy = 	0;
		const static int Pinger = 	1;
		const static int Pipe = 	2;
	};

	Landmark();
	Landmark(cv::Point3d center, double orientation, cv::Point3d dim, int shapeType);

	cv::Point3d mCenter;
	double mOrientation;
	cv::Point3d mDim;
	cv::Vec3b mColor;

	int mShapeType; //how to draw this object
	int mLandmarkId;
};

namespace LandmarkTypes
{
	class Buoy : public Landmark
	{
	public:
		Buoy(cv::Point3d center, double orientation, cv::Point3d dim, cv::Vec3b color);	
	};

	class Pinger : public Landmark
	{
	public:
		Pinger(cv::Point3d center, double orientation, cv::Point3d dim, int id);
		
		int mId;
	};

	class Pipe : public Landmark
	{
	public:
		Pipe(cv::Point3d center, double orientation, cv::Point3d dim);
	};
}
