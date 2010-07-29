#include <ros/ros.h>

#include <list>
#include <opencv/cv.h>
#include <opencv/highgui.h>

using namespace cv;

//Assuming External Pressure sensor readings
class depth_meter
{
public:
  depth_meter();
  depth_meter(int width, int height);
  int getMinDrawDepth(int d);
  IplImage* render(int d);

  int itsWidth, itsHeight;
private:
  CvFont itsFont;
  int itsCurrentDepth;
  std::list<int> itsDepthHist;
};
