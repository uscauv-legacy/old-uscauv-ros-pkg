#include <landmark_map/Landmark.h>

Landmark::Landmark()
{

}

Landmark::Landmark(cv::Point3d center, double orientation, cv::Point3d dim, int shapeType)
{
	mCenter = center;
	mOrientation = orientation;
	mDim = dim;
	mShapeType = shapeType;
}

LandmarkTypes::Buoy::Buoy(cv::Point3d center, double orientation, cv::Point3d dim, cv::Vec3b color) : 
Landmark(center, orientation, dim, Landmark::ShapeType::sphere)
{
	mColor = color;
	mLandmarkId = LandmarkId::Buoy;
}

LandmarkTypes::Pinger::Pinger(cv::Point3d center, double orientation, cv::Point3d dim, int id) : 
Landmark(center, orientation, dim, Landmark::ShapeType::cylinder), mId(id)
{
	mColor = cv::Vec3b( 0, 255, 0 );
	mLandmarkId = LandmarkId::Pinger;
}

LandmarkTypes::Pipe::Pipe(cv::Point3d center, double orientation, cv::Point3d dim) : 
Landmark(center, orientation, dim, Landmark::ShapeType::cube)
{
	mColor = cv::Vec3b( 255, 127, 0 );
	mLandmarkId = LandmarkId::Pipe;
}
