#include <cstddef>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <iostream>

#include "CVSuperPixel.H"

int main()
{

  std::cout << "Opening Camera ";
  cv::VideoCapture cap(0);
  std::cout << "Done" << std::endl;

  if(!cap.isOpened())
  {
    std::cerr << "Cannot open video source" << std::endl;
    return -1;
  }

  int sigma = 3;
  int c = 500;
  int min_size = 100;
  cv::namedWindow("Debug");
  cv::createTrackbar("Sigma",   "Debug", &sigma,    20);
  cv::createTrackbar("C",       "Debug", &c,        1000);
  cv::createTrackbar("MinSize", "Debug", &min_size, 1000);


  while(1)
  {
    if(sigma == 0)
    { std::cerr << "Keep sigma above 0, you dummy" << std::endl; continue; }

    cv::Mat inputImage;
    cap >> inputImage;

    cv::Mat smallImage;

    float scale = .25;
    cv::resize(inputImage, smallImage,
      cv::Size(inputImage.size().width*scale, inputImage.size().height*scale));

    std::vector<std::vector<cv::Point> > groups = 
      SuperPixelSegment(smallImage, sigma, c, min_size);

    std::cout << "Got " << groups.size() << " groups" << std::endl;

    cv::Mat debugImage = SuperPixelDebugImage(groups, smallImage);

    cv::Mat displayImage;
    cv::resize(debugImage, displayImage,
      cv::Size(debugImage.size().width/scale, debugImage.size().height/scale));

    cv::imshow("Debug", displayImage);
    cv::waitKey(5);
  }

  return 0;
}

