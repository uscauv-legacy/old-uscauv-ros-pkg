#include <cstddef>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <iostream>

#include "CVSuperPixel.H"

// ######################################################################
// Parameters
// ######################################################################
int sigma     = 3;
int c         = 500;
int min_size  = 100;
int hMean     = 0;
int sMean     = 0;
int vMean     = 0;
int hWeight   = 0;
int sWeight   = 0;
int vWeight   = 0;
int threshold = 0;
int scale     = 25;

// ######################################################################
void saveSettings(int val, void* data)
{
  std::cout << "saving data" << std::endl;

  cv::FileStorage colorfile("colors.yaml", cv::FileStorage::WRITE);
  if(colorfile.isOpened())
  {

    colorfile << "Sigma "    << sigma;
    colorfile << "C"         <<  c;
    colorfile << "MinSize"   << min_size;
    colorfile << "HueMean"   << hMean;       
    colorfile << "SatMean"   << sMean;
    colorfile << "ValMean"   << vMean;
    colorfile << "HueWeight" << hWeight;  
    colorfile << "SatWeight" << sWeight;  
    colorfile << "ValWeight" << vWeight;  
    colorfile << "Threshold" << threshold;  
    colorfile << "Scale"     << scale;  
    colorfile.release();
  }
  else
  {
    std::cerr << " COULD NOT WRITE TO SETTINGS FILE!" << std::endl;
  }
};

// ######################################################################
void loadSettings()
{
  cv::FileStorage colorfile("colors.yaml", cv::FileStorage::READ);
  if(colorfile.isOpened())
  {
    std::cout << "Loading Defaults..." << std::endl;

    sigma     = int(colorfile["Sigma"]);     
    c         = int(colorfile["C"]);     
    min_size  = int(colorfile["MinSize"]);     
    hMean     = int(colorfile["HueMean"]);     
    hWeight   = int(colorfile["HueWeight"]);
    sMean     = int(colorfile["SatMean"]);     
    sWeight   = int(colorfile["SatWeight"]);
    vMean     = int(colorfile["ValMean"]);     
    vWeight   = int(colorfile["ValWeight"]);
    threshold = int(colorfile["Threshold"]);
    scale     = int(colorfile["Scale"]);
    colorfile.release();
  }
  else
  {
    std::cerr << "No defaults found... " << std::endl;
  }
  
}

// ######################################################################
int main()
{
  // Load the settings from the yaml file
  loadSettings();

  std::cout << "Opening Camera ";
  cv::VideoCapture cap(0);
  std::cout << "Done" << std::endl;

  if(!cap.isOpened())
  {
    std::cerr << "Cannot open video source" << std::endl;
    return -1;
  }

  // Setup the controls
  int dummy;
  cv::namedWindow("Debug");
  cv::namedWindow("Distance");
  cv::createTrackbar("Sigma",         "Debug", &sigma,    20);
  cv::createTrackbar("Image Scale %", "Debug", &scale, 100);
  cv::createTrackbar("MinSize",       "Debug", &min_size, 1000);
  cv::createTrackbar("C",             "Debug", &c,        1000);
  cv::createTrackbar("Save",          "Debug", &dummy, 1, &saveSettings);
  cv::createTrackbar("H",             "Distance", &hMean, 255);
  cv::createTrackbar("H Weight",      "Distance", &hWeight, 255);
  cv::createTrackbar("S",             "Distance", &sMean, 255);
  cv::createTrackbar("S Weight",      "Distance", &sWeight, 255);
  cv::createTrackbar("V",             "Distance", &vMean, 255);
  cv::createTrackbar("V Weight",      "Distance", &vWeight, 255);
  cv::createTrackbar("Threshold",     "Distance", &threshold, 255);

  while(1)
  {
    // Read an image from the camera
    cv::Mat inputImage;
    cap >> inputImage;

    float normScale = float(scale)/100.0;
    // Resize the image before we do any segmentation to speed things up
    cv::Mat smallImage;
    cv::resize(inputImage, smallImage,
        cv::Size(inputImage.size().width*normScale, inputImage.size().height*normScale));

    // Segment the image using Felzenszwalb's graph-based segmentation algorithm
    std::vector<std::vector<cv::Point> > groups = 
      SuperPixelSegment(smallImage, sigma, c, min_size);

    // Find the distance in color space to the desired color for each of our segmented groups
    cv::Mat smallHSVImage;
    cv::cvtColor(smallImage, smallHSVImage, CV_BGR2HSV);
    cv::Mat distImage(smallImage.size(), CV_32F);
    std::vector<std::vector<cv::Point> > buoyGroups;
    for(size_t grpIdx=0; grpIdx < groups.size(); grpIdx++)
    {
      cv::Vec3f avgColor(0,0,0);
      for(size_t pntIdx=0; pntIdx<groups[grpIdx].size(); pntIdx++)
        avgColor += smallHSVImage.at<cv::Vec3b>(groups[grpIdx][pntIdx]);
      avgColor[0] /= groups[grpIdx].size();
      avgColor[1] /= groups[grpIdx].size();
      avgColor[2] /= groups[grpIdx].size();

      // Find the weighted distance to our desired color
      float dist = 
        abs(hMean - avgColor[0]) / float(255 - hWeight)/10.0
        +
        abs(sMean - avgColor[1]) / float(255 - sWeight)/10.0
        +
        abs(vMean - avgColor[2]) / float(255 - vWeight)/10.0;

      for(size_t pntIdx=0; pntIdx<groups[grpIdx].size(); pntIdx++)
        distImage.at<float>(groups[grpIdx][pntIdx]) = dist;

      // If this group looks cool then push it back into the list of potential buoy groups
      float normThresh = threshold / 1000.0;
      std::cout << " Dist: " << dist << " Thresh: " << normThresh << std::endl;
      if(dist < normThresh) buoyGroups.push_back(groups[grpIdx]);
    }

    // Make a mean looking debug image
    cv::Mat debugImage = SuperPixelDebugImage(groups, smallImage);

    // Resize the images for display
    cv::Mat displayImage;
    cv::resize(debugImage, displayImage,
        cv::Size(debugImage.size().width/normScale, debugImage.size().height/normScale));
    cv::Mat bigDistImage;
    cv::resize(distImage, bigDistImage,
        cv::Size(debugImage.size().width/normScale, debugImage.size().height/normScale));

    // Go through all of the buoy groups and find the bounding boxes around
    // each, then draw that box on the displayImage. Edward: this is the data you should post
    std::cout << buoyGroups.size() << " buoys detected" << std::endl;
    for(size_t grpIdx=0; grpIdx < buoyGroups.size(); ++grpIdx)
    {
      int minX = inputImage.size().width;
      int maxX = 0;
      int minY = inputImage.size().height;
      int maxY = 0;

      for(size_t pntIdx=0; pntIdx < buoyGroups[grpIdx].size(); ++pntIdx)
      {
        minX = std::min(minX, buoyGroups[grpIdx][pntIdx].x);
        maxX = std::max(maxX, buoyGroups[grpIdx][pntIdx].x);
        minY = std::min(minY, buoyGroups[grpIdx][pntIdx].y);
        maxY = std::max(maxY, buoyGroups[grpIdx][pntIdx].y);
      }
      cv::rectangle(displayImage, cv::Point(minX/normScale, minY/normScale), cv::Point(maxX/normScale, maxY/normScale),
        255, 2);
    }

    // Show it!
    cv::imshow("Debug", displayImage);
    cv::imshow("Distance", bigDistImage);
    cv::waitKey(50);
  }

  return 0;
}

