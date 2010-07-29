/*******************************************************************************
 *
 *      rectangle_finder
 * 
 *      Copyright (c) 2010, Michael Wei, Edward T. Kaszubski (ekaszubski@gmail.com)
 *      All rights reserved.
 *
 *      Redistribution and use in source and binary forms, with or without
 *      modification, are permitted provided that the following conditions are
 *      met:
 *      
 *      * Redistributions of source code must retain the above copyright
 *        notice, this list of conditions and the following disclaimer.
 *      * Redistributions in binary form must reproduce the above
 *        copyright notice, this list of conditions and the following disclaimer
 *        in the documentation and/or other materials provided with the
 *        distribution.
 *      * Neither the name of the USC Underwater Robotics Team nor the names of its
 *        contributors may be used to endorse or promote products derived from
 *        this software without specific prior written permission.
 *      
 *      THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *      "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *      LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *      A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *      OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *      SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *      LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *      DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *      THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *      (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *      OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *******************************************************************************/

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Point.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/CvBridge.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <vector>
#include <utility>
#include <cmath>

#include <rectangle_finder/RectangleArrayMsg.h>
#include <rectangle_finder/FindRectangles.h>

#define MIN_CENTER_DIST		15
#define MIN_AREA		150
#define CORNER_TOLERANCE	4

int thresh = 42;
double angle_thresh = 0.4;
uint itsWidth = 320;
uint itsHeight = 240;

sensor_msgs::ImageConstPtr itsCurrentImage;
//boost::mutex image_mutex;

image_transport::Publisher * rectangleImagePub;
sensor_msgs::CvBridge bridge;

int debug_mode;

class Point2D
{
public:
	Point2D() : i(0), j(0) {}
	Point2D(int _i, int _j) : i(_i), j(_j){}
	int i;
	int j;
};

class Quadrilateral
{
public:
	Quadrilateral() : ratio(0.0), angle(0.0){}
	float ratio;
	float angle;
	Point2D tl;
	Point2D tr;
	Point2D bl;
	Point2D br;
	Point2D center;
};

class LineSegment2D
{
public:
	LineSegment2D();
	LineSegment2D(Point2D a, Point2D b) : point1(a), point2(b){}
	Point2D point1;
	Point2D point2;
};

float distance(Point2D p1, Point2D p2)
{
	int d1 = p1.i - p2.i;
	int d2 = p1.j - p2.j;
	return sqrt(float(d1 * d1 + d2 * d2));
}

float lineLength(LineSegment2D line)
{
	return distance(line.point1, line.point2);	
}

double lineAngle(LineSegment2D line)
{
	float o = (float)(line.point1.j - line.point2.j);
	float a = (float)(line.point1.i - line.point2.i);
	return atan(o/a);
}

// helper function:
// finds a cosine of angle between vectors
// from pt0->pt1 and from pt0->pt2
double angle(CvPoint* pt1, CvPoint* pt2, CvPoint* pt0)
{
	double dx1 = pt1->x - pt0->x;
	double dy1 = pt1->y - pt0->y;
	double dx2 = pt2->x - pt0->x;
	double dy2 = pt2->y - pt0->y;
	return (dx1*dx2 + dy1*dy2)/sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
}


bool inCorner(CvSeq* result)
{
	for (int pti = 0; pti < result->total; pti++)
	{
		uint i = ((CvPoint*)cvGetSeqElem(result, pti))->x;
		uint j = ((CvPoint*)cvGetSeqElem(result, pti))->y;

		//edge contours are inset by 1
		uint right_edge = itsWidth - 2;
		uint bottom_edge = itsHeight - 2;

		if((i <= CORNER_TOLERANCE && j <= CORNER_TOLERANCE) ||
			(i >= right_edge-CORNER_TOLERANCE && j <= CORNER_TOLERANCE) ||
			(i <= CORNER_TOLERANCE && j >= bottom_edge-CORNER_TOLERANCE) ||
			(i >= right_edge-CORNER_TOLERANCE && j >= bottom_edge-CORNER_TOLERANCE))
		{
			return true;
		}
	}
	return false;
}


// returns sequence of squares detected on the image.
// the sequence is stored in the specified memory storage
CvSeq* findSquares4(IplImage* img, CvMemStorage* storage)
{
	CvSize sz = cvSize( img->width & -2, img->height & -2 );
	CvSeq* contours;
	CvSeq* result;
	int i, c, l, N = 11;
	double s, t;
	double img_area = sz.width * sz.height;
	double max_area = img_area * 0.5;

	IplImage* timg = cvCloneImage( img ); // make a copy of input image
	IplImage* gray = cvCreateImage( sz, 8, 1 );
	IplImage* pyr = cvCreateImage( cvSize(sz.width/2, sz.height/2), 8, 3 );
	IplImage* tgray;

	// create empty sequence that will contain points -
	// 4 points per square (the square's vertices)
	CvSeq* squares = cvCreateSeq( 0, sizeof(CvSeq), sizeof(CvPoint), storage );

	// select the maximum ROI in the image
	// with the width and height divisible by 2
	//cvSetImageROI( timg, cvRect( max_roi_x1, max_roi_y1, max_roi_w, max_roi_h ));
	cvSetImageROI( timg, cvRect( 0, 0, sz.width, sz.height ));

	// down-scale and upscale the image to filter out the noise
	cvPyrDown( timg, pyr, 7 );
	cvPyrUp( pyr, timg, 7 );
	tgray = cvCreateImage( sz, 8, 1 );

	// TAKE OUT LATER
	//	bottom 2 commented out by Mike Wei for conversion to ROS. 
	//itsDisp.clear();
	//inplacePaste(itsDisp, ipl2rgb(img), Point2D<int>(0, 0));

	// find squares in every color plane of the image
	for( c = 0; c < 3; c++ )
	{
		// extract the c-th color plane
		cvSetImageCOI( timg, c+1 );
		cvCopy( timg, tgray, 0 );

		// try several threshold levels
		for( l = 0; l < N; l++ )
		{
			// hack: use Canny instead of zero threshold level.
			// Canny helps to catch squares with gradient shading
			if( l == 0 )
			{
				// apply Canny. Take the upper threshold from slider
				// and set the lower to 0 (which forces edges merging)
				cvCanny( tgray, gray, 0, thresh, 5 );

				// dilate canny output to remove potential
				// holes between edge segments
				cvDilate( gray, gray, 0, 1 );
			} else {
				// apply threshold if l!=0:
				// tgray(x,y) = gray(x,y) < (l+1)*255/N ? 255 : 0
				cvThreshold( tgray, gray, (l+1)*255/N, 255, CV_THRESH_BINARY );
			}

			// find contours and store them all as a list
			cvFindContours(gray, storage, &contours, sizeof(CvContour), CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, cvPoint(0,0));

			// test each contour
			uint count =0;
			while( contours ) {

				// approximate contour with accuracy proportional
				// to the contour perimeter
				result = cvApproxPoly( contours, sizeof(CvContour), storage, CV_POLY_APPROX_DP, cvContourPerimeter(contours)*0.02, 0 );

				// square contours should have 4 vertices after approximation
				// relatively large area (to filter out noisy contours)
				// and be convex.
				// Note: absolute value of an area is used because
				// area may be positive or negative - in accordance with the
				// contour orientation

				//also going to check that the area is smaller than max_area
				double rect_area = fabs(cvContourArea(result,CV_WHOLE_SEQ));

				if (result->total == 4
            && rect_area > MIN_AREA
            && rect_area < max_area
            && cvCheckContourConvexity(result)
            && !inCorner(result))
				{
					s = 0;

					for (i = 0; i < 5; i++)
					{
						// find minimum angle between joint
						// edges (maximum of cosine)
						if (i >= 2)
						{
							t = fabs(angle(
									(CvPoint*)cvGetSeqElem( result, i ),
									(CvPoint*)cvGetSeqElem( result, i-2 ),
									(CvPoint*)cvGetSeqElem( result, i-1 )));

							s = s > t ? s : t;
						}
					}

					// if cosines of all angles are small
					// (all angles are ~90 degree) then write quandrange
					// vertices to resultant sequence
					if (s < angle_thresh)
					{
						//LINFO("RECTANGLE BABY");
						for (i = 0; i < 4; i++)
						{
							cvSeqPush( squares, (CvPoint*)cvGetSeqElem( result, i ));
						}
					}
				}

				// take the next contour
				contours = contours->h_next;
				count++;
			}
		}
	}

	// release all the temporary images
	cvReleaseImage( &gray );
	cvReleaseImage( &pyr );
	cvReleaseImage( &tgray );
	cvReleaseImage( &timg );

	return squares;
}

//void findRectangle(const sensor_msgs::ImageConstPtr &img)
rectangle_finder::RectangleArrayMsg findRectangles()
{
	rectangle_finder::RectangleArrayMsg rectangles;
	CvMemStorage* storage = cvCreateMemStorage(0); 
	//IplImage* img0 = bridge.imgMsgToCv(img);
	IplImage * img0 = bridge.imgMsgToCv(itsCurrentImage);
	CvSeq *cvsquares = findSquares4( img0, storage );

	// Release the image later after we have drawn on it
	// cvReleaseImage( &img0 );

	std::vector<Quadrilateral> quadVect;
	Quadrilateral quad;
	Point2D avgCenter;
	std::multimap<int, Point2D> tempPoints;

	// iterate over all the quadrilateral points found
	for(int i=0; i < cvsquares->total; i++)
	{
		// get an individual point
		Point2D quadPoint;
		quadPoint.i = ((CvPoint*)cvGetSeqElem(cvsquares, i))->x;
		quadPoint.j = ((CvPoint*)cvGetSeqElem(cvsquares, i))->y;
		
		// add current point's position to running average of point positions
		//avgCenter += Point2D<int>(quadPoint.i,quadPoint.j);
		avgCenter.i += quadPoint.i;
		avgCenter.j += quadPoint.j;

		// add the point to map, sorted by point Y-axis position
		tempPoints.insert(std::make_pair(quadPoint.j,quadPoint));
		//LINFO("tempPoints size: %d\n",tempPoints.size());
		// if we have added the 4th point on the quadrilateral
		if (tempPoints.size() == 4)
		{
			std::vector<Point2D> tempVec;

			for(std::map<int,Point2D>::const_iterator it = tempPoints.begin();
					it != tempPoints.end(); ++it)
			{
				tempVec.push_back(it->second);
			}

			// compare first two points to determine which is top left
			// and which is top right
			if(tempVec[0].i < tempVec[1].i)
			{
				quad.tl.i = tempVec[0].i;
				quad.tr.i = tempVec[1].i;

				quad.tl.j = tempVec[0].j;
				quad.tr.j = tempVec[1].j;
			}
			else
			{
				quad.tr.i = tempVec[0].i;
				quad.tl.i = tempVec[1].i;
               
				quad.tr.j = tempVec[0].j;
				quad.tl.j = tempVec[1].j;
			}

			// compare second two points to determine bottom left and
			// bottom right
			if(tempVec[2].i < tempVec[3].i)
			{
				quad.bl.i = tempVec[2].i;
				quad.br.i = tempVec[3].i;

				quad.bl.j = tempVec[2].j;
				quad.br.j = tempVec[3].j;
			}
			else
			{
				quad.br.i = tempVec[2].i;
				quad.bl.i = tempVec[3].i;

				quad.br.j = tempVec[2].j;
				quad.bl.j = tempVec[3].j;
			}


			// divide by total number of averaged points
			// to get current quad's center position
			//avgCenter /= Point2D<int>(4,4);
			avgCenter.i /= 4;
			avgCenter.j /= 4;
			quad.center.i = avgCenter.i;
			quad.center.j = avgCenter.j;

			bool isDupe = false;

			// make sure the quad's center is not too close
			// to a prev. quad's center in order to avoid duplicates
			for(uint j = 0; j < quadVect.size(); j++)
			{
				Point2D temp = Point2D(quadVect[j].center.i, quadVect[j].center.j);
				//if(avgCenter.distance(Point2D<int>(quadVect[j].center.i,quadVect[j].center.j))
				//	< MIN_CENTER_DIST)
				if (distance(avgCenter, temp) < MIN_CENTER_DIST)
				{
					isDupe = true;
				}
			}

			// not dupe so add it to vector
			if(!isDupe)
			{
				Point2D v1 = Point2D((quad.tr.i + quad.tl.i)/2, (quad.tr.j + quad.tl.j)/2);
				Point2D v2 = Point2D((quad.br.i + quad.bl.i)/2, (quad.br.j + quad.bl.j)/2);
				Point2D h1 = Point2D((quad.tl.i + quad.bl.i)/2, (quad.tl.j + quad.bl.j)/2);
				Point2D h2 = Point2D((quad.tr.i + quad.br.i)/2, (quad.tr.j + quad.br.j)/2);
				LineSegment2D vertLine = LineSegment2D(v1, v2);
				LineSegment2D horizLine = LineSegment2D(h1, h2);

				float ratio = 0.0;
				float angle = 0.0;

				if(lineLength(vertLine) > lineLength(horizLine))
				{
					if(lineLength(horizLine) > 0)
						ratio = lineLength(vertLine) / lineLength(horizLine);

					angle = lineAngle(vertLine);
				}
				else
				{
					if(lineLength(vertLine) > 0)
						ratio = lineLength(horizLine) / lineLength(vertLine);

					angle = lineAngle(horizLine);
				}

				// change angle to degrees
				angle = angle * (180/M_PI);

				// normalize angle so that zero degrees is facing forawrd
				// turning to the right is [0 -> 90]
				// turning to the left is [0 -> -90]
				if(angle < 0)
					angle += 90;
				else
					angle += -90;

				quad.ratio = ratio;
				quad.angle = angle;
				quadVect.push_back(quad);
				
				if(debug_mode == 1)
				{
					// draw the quad on the image
					cvLine(img0, cv::Point(quad.tr.i, quad.tr.j),
						cv::Point(quad.br.i, quad.br.j), cvScalar(0, 255, 0), 2);
					cvLine(img0, cv::Point(quad.br.i, quad.br.j),
						cv::Point(quad.bl.i, quad.bl.j), cvScalar(0, 255, 0), 2);
					cvLine(img0, cv::Point(quad.bl.i, quad.bl.j),
						cv::Point(quad.tl.i, quad.tl.j), cvScalar(0, 255, 0), 2);
					cvLine(img0, cv::Point(quad.tl.i, quad.tl.j),
						cv::Point(quad.tr.i, quad.tr.j), cvScalar(0, 255, 0), 2);
				}
			}

			// re-initialize for next quad
			quad = Quadrilateral();
			avgCenter = Point2D();
			tempPoints.clear();
		}
	}

	if (quadVect.size() > 0)
	{
		for (uint a = 0; a < quadVect.size(); a++)
		{
			rectangle_finder::RectangleMsg msg;
			msg.Center.x = quadVect[a].center.i;
			msg.Center.y = quadVect[a].center.j;
			msg.Center.z = 0.0;
			
			/// ToDo: Implement rectangle size calculation
			
			msg.Dim.x = quadVect[a].tr.j - quadVect[a].tl.j;
			msg.Dim.y = quadVect[a].tr.i - quadVect[a].br.i;
			msg.Dim.z = 0.0;
			
			/// ToDo: Implement rectangle orientation calculation
			
			msg.Ori = quadVect[a].angle;
			
			rectangles.RectangleArray.push_back(msg);
		}
		if(debug_mode == 1)
		{
			rectangleImagePub->publish(bridge.cvToImgMsg(img0));
		}
	}

	cvReleaseMemStorage(&storage);
	//cvReleaseImage(&img0);
	return rectangles;
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	//boost::mutex::scoped_lock lock(image_mutex); 
	itsCurrentImage = msg; //bridge.imgMsgToCv(msg);
  
	if(debug_mode == 1)
		findRectangles();  
}

bool FindRectanglesCallback(rectangle_finder::FindRectangles::Request & req, rectangle_finder::FindRectangles::Response & resp)
{
	resp.Rectangles = findRectangles();
	
	if(resp.Rectangles.RectangleArray.size() > 0)
	{
		return true;
	}
	return false;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "rectangle_finder");
	ros::NodeHandle n("~");
	image_transport::ImageTransport it(n);
	
	n.param("debug_mode", debug_mode, 0);
	
	// Register publisher for the center of the bin position in an image
	ros::ServiceServer rectangle_srv = n.advertiseService("FindRectangles", FindRectanglesCallback);

	// Register publisher for image with bins highlighted
	rectangleImagePub = new image_transport::Publisher(it.advertise("debug/image_color", 1));

	// Subscribe to an image topic
	image_transport::Subscriber sub = it.subscribe("image", 1, imageCallback);

	ros::spin();
}

