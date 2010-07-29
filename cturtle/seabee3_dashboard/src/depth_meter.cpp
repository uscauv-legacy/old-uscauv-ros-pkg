#include "depth_meter.h"

#define READING_AVG 1
#define MIN_DRAW_BOUND 0.0
#define MAX_DRAW_BOUND 1.0

using namespace cv;

depth_meter::depth_meter():
  itsWidth(0),
  itsHeight(0)

{
    		double hScale=0.3;
    		double vScale=0.3;
    		int lineWidth=1;

    		cvInitFont(&itsFont,CV_FONT_HERSHEY_SIMPLEX, hScale,vScale,0,lineWidth);
}

depth_meter::depth_meter(int width, int height):
  itsWidth(width),
  itsHeight(height)
{
    		double hScale=0.3;
    		double vScale=0.3;
    		int lineWidth=1;

    		cvInitFont(&itsFont,CV_FONT_HERSHEY_SIMPLEX, hScale,vScale,0,lineWidth);
}

int depth_meter::getMinDrawDepth(int d)
{
  int dFrac = d % 100;
  
  if(dFrac == 0) return 0;

  int dQuart = 0;

  if(dFrac <= 25)
    dQuart = 25 - dFrac;
  else if(dFrac > 25 && dFrac <= 50)
    dQuart = 50 - dFrac;
  else if(dFrac > 50 && dFrac <= 75)
    dQuart = 75 - dFrac;
  else if(dFrac > 75)
    dQuart = 100 - dFrac;

  return dQuart;
}

IplImage* depth_meter::render(int d)
{
  itsDepthHist.push_front(d);
  if(itsDepthHist.size() > READING_AVG)
    itsDepthHist.pop_back();

  int avgDepth = 0;

  std::list<int>::reverse_iterator it = itsDepthHist.rbegin();
  for(;it != itsDepthHist.rend(); ++it)
    {
      avgDepth += *it;
    }
  if(itsDepthHist.size() > 0)
    avgDepth /= itsDepthHist.size();

  IplImage* depthImage = cvCreateImage(cvSize(itsWidth,itsHeight),
				       IPL_DEPTH_32F,3);
  
  int minDepth = avgDepth - itsHeight/2;//getMinDrawDepth(avgDepth);
  
  minDepth = getMinDrawDepth(minDepth);

  //minDepth = (minDepth >= MIN_DRAW_BOUND*itsHeight) ? 
  //  minDepth : getMinDrawDepth( MIN_DRAW_BOUND*itsHeight);
  
  // draw current depth text
  cvPutText(depthImage,
	    boost::lexical_cast<std::string>(avgDepth).c_str(),
	    cvPoint((int)(.80*(float)depthImage->width),
		    itsHeight/2),
	    &itsFont,
	    CV_RGB(20,253,0)
	    );
  
  // draw tick marks
  for(int drawHeight = minDepth; 
      drawHeight <= (int)(MAX_DRAW_BOUND*itsHeight)-1; 
      drawHeight += 25)
    {
      int cDepth = (drawHeight + avgDepth - itsHeight/2);
      int drawDec = (cDepth%100);

      // draw tick mark text
      cvPutText(depthImage,
		boost::lexical_cast<std::string>(cDepth).c_str(),
		cvPoint((int)(.05*(float)depthImage->width),
			drawHeight+3),
		&itsFont,
		CV_RGB(20,253,0)
		);
      
      if(drawDec == 25 || drawDec == 75)
	{
	  cvLine(depthImage,
		 cvPoint((int)(.40*(float)depthImage->width),(int)drawHeight),
		 cvPoint((int)(.60*(float)depthImage->width),(int)drawHeight),
		 CV_RGB(20,253,0)
		 );
	}
      else if(drawDec == 50)
	{
	  cvLine(depthImage,
		 cvPoint((int)(.30*(float)depthImage->width),(int)drawHeight),
		 cvPoint((int)(.70*(float)depthImage->width),(int)drawHeight),
		 CV_RGB(20,253,0)
		 );
	}
      else
	{
	  cvLine(depthImage,
		 cvPoint((int)(.25*(float)depthImage->width),(int)drawHeight),
		 cvPoint((int)(.75*(float)depthImage->width),(int)drawHeight),
		 CV_RGB(20,253,0)
		 );
	}
    }

  // draw current depth line as horizontal over middle of image
  cvLine(depthImage, cvPoint((int)(0.0*(float)depthImage->width),
			    (depthImage->height/2)-10), 
	 cvPoint(depthImage->width,
		 (depthImage->height/2)-10),
	 CV_RGB(255,0,0));

  cvLine(depthImage, cvPoint((int)(0.0*(float)depthImage->width),
			    (depthImage->height/2+4)), 
	 cvPoint(depthImage->width,
		 (depthImage->height/2 + 4)),
	 CV_RGB(255,0,0));

  return depthImage;
}
