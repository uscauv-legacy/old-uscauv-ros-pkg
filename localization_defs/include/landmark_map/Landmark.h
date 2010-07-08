#include <opencv/cv.h>
#include <visualization_msgs/Marker.h>
#include <localization_defs/LandmarkMsg.h>
#include <string>

class Landmark
{
public:
	
	struct LandmarkType
	{
		const static int NumTypes =	3;
		
		const static int None =		-1;
		const static int Buoy = 	0;
		const static int Pinger = 	1;
		const static int Pipe = 	2;
		const static int Bin = 		3;
		const static int Window = 	4;
	};
	
	struct ColorIds
	{
		const static int red		= 0 ;
		const static int lred		= 1 ;
		const static int orange		= 2 ;
		const static int yellow		= 3 ;
		const static int lrgreen	= 4 ;
		const static int green		= 5 ;
		const static int lgreen		= 6 ;
		const static int lbgreen	= 7 ;
		const static int blugreen	= 8 ;
		const static int lgblue		= 9 ;
		const static int blue		= 10;
		const static int lblue		= 11;
		const static int lrblue		= 12;
		const static int purple		= 13;
		const static int pink		= 14;
		const static int black		= 15;
	};
	
	struct ColorDefs
	{	
		static cv::Vec3b getColor(int colorId)
		{
			switch(colorId)
			{
			case ColorIds::red :		return cv::Vec3b (255, 0  , 0  );
			case ColorIds::lred :		return cv::Vec3b (255, 127, 127);
			case ColorIds::orange :		return cv::Vec3b (255, 127, 0  );
			case ColorIds::yellow :		return cv::Vec3b (255, 255, 0  );
			case ColorIds::lrgreen :	return cv::Vec3b (127, 255, 0  );
			case ColorIds::green :		return cv::Vec3b (0  , 255, 0  );
			case ColorIds::lgreen :		return cv::Vec3b (127, 255, 127);
			case ColorIds::lbgreen :	return cv::Vec3b (0  , 255, 127);
			case ColorIds::blugreen :	return cv::Vec3b (0  , 255, 255);
			case ColorIds::lgblue :		return cv::Vec3b (0  , 127, 255);
			case ColorIds::blue :		return cv::Vec3b (0  , 0  , 255);
			case ColorIds::lblue :		return cv::Vec3b (127, 127, 255);
			case ColorIds::lrblue :		return cv::Vec3b (127, 0  , 255);
			case ColorIds::purple :		return cv::Vec3b (255, 0  , 255);
			case ColorIds::pink :		return cv::Vec3b (255, 0  , 127);
			case ColorIds::black : 		return cv::Vec3b (0  , 0  , 0  );
			}
			return cv::Vec3b (0, 0, 0);
		}
	};

	Landmark();
	Landmark(cv::Point3d center, double orientation, cv::Point3d dim, int shapeType);
	
	visualization_msgs::Marker createMarker(const std::string & frame, const int id) const;
	localization_defs::LandmarkMsg createMsg() const;

	cv::Point3d mCenter; //position relative to the center of the map (which is always the origin, <0, 0, 0>)
	double mOrientation;
	cv::Point3d mDim;
	int mColor;

	int mShapeType; //how to draw this object
	int mLandmarkType;
	int mId;
};

namespace LandmarkTypes
{
	class Buoy : public Landmark
	{
	public:
		Buoy(cv::Point3d center, double orientation, int color);
//		visualization_msgs::Marker createMarker(std::string ns, std::string frame);
	};

	class Pinger : public Landmark
	{
	public:
		Pinger(cv::Point3d center, double orientation, int id);
//		visualization_msgs::Marker createMarker(std::string ns, std::string frame);
	};

	class Pipe : public Landmark
	{
	public:
		Pipe(cv::Point3d center, double orientation);
//		visualization_msgs::Marker createMarker(std::string ns, std::string frame);
	};
}
