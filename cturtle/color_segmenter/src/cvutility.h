#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include <opencv/highgui.h>
#include <vector>

using namespace cv;
using namespace std;

IplImage* mdmFilterHS(Mat src,int Hmin,int Hmax, int Smin, int Smax);
IplImage* cvFilterHS(IplImage* src,int Hmin,int Hmax, int Smin, int Smax, int mode);
void cvFlipBinaryImg(IplImage* src);
IplImage* cvFilterRGB(IplImage* src,int Rmin,int Rmax, int Gmin, int Gmax,int Bmin, int Bmax, int mode);
