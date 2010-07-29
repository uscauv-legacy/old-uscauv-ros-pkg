#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Vector3.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/CvBridge.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <queue>
#include <vector>
#include <utility>
#include <cmath>
//#include "stereo_vision/ResetPosition.h"


// I just made these up
#define FOCAL_LENGTH 0.005 // meters
#define CAMERA_DISTANCE 0.8	// meters

using namespace std;

ros::Publisher svPub;
//ros::ServiceServer service;
image_transport::Publisher svImagePub;
sensor_msgs::CvBridge bridge;

queue<IplImage*> leftImages;
queue<IplImage*> rightImages;
//geometry_msgs::Point currentPosition;


double compareSURFDescriptors( const float* d1, const float* d2, double best, int length )
{
    double total_cost = 0;
    assert( length % 4 == 0 );
    for( int i = 0; i < length; i += 4 )
    {
        double t0 = d1[i] - d2[i];
        double t1 = d1[i+1] - d2[i+1];
        double t2 = d1[i+2] - d2[i+2];
        double t3 = d1[i+3] - d2[i+3];
        total_cost += t0*t0 + t1*t1 + t2*t2 + t3*t3;
        if( total_cost > best )
            break;
    }
    return total_cost;
}


int naiveNearestNeighbor( const float* vec, int laplacian,
                      const CvSeq* model_keypoints,
                      const CvSeq* model_descriptors )
{
    int length = (int)(model_descriptors->elem_size/sizeof(float));
    int i, neighbor = -1;
    double d, dist1 = 1e6, dist2 = 1e6;
    CvSeqReader reader, kreader;
    cvStartReadSeq( model_keypoints, &kreader, 0 );
    cvStartReadSeq( model_descriptors, &reader, 0 );

    for( i = 0; i < model_descriptors->total; i++ )
    {
        const CvSURFPoint* kp = (const CvSURFPoint*)kreader.ptr;
        const float* mvec = (const float*)reader.ptr;
    	CV_NEXT_SEQ_ELEM( kreader.seq->elem_size, kreader );
        CV_NEXT_SEQ_ELEM( reader.seq->elem_size, reader );
        if( laplacian != kp->laplacian )
            continue;
        d = compareSURFDescriptors( vec, mvec, dist2, length );
        if( d < dist1 )
        {
            dist2 = dist1;
            dist1 = d;
            neighbor = i;
        }
        else if ( d < dist2 )
            dist2 = d;
    }
    if ( dist1 < 0.6*dist2 )
        return neighbor;
    return -1;
}


void findPairs( const CvSeq* objectKeypoints, const CvSeq* objectDescriptors,
           const CvSeq* imageKeypoints, const CvSeq* imageDescriptors, vector<int>& ptpairs )
{
    int i;
    CvSeqReader reader, kreader;
    cvStartReadSeq( objectKeypoints, &kreader );
    cvStartReadSeq( objectDescriptors, &reader );
    ptpairs.clear();

    for( i = 0; i < objectDescriptors->total; i++ )
    {
        const CvSURFPoint* kp = (const CvSURFPoint*)kreader.ptr;
        const float* descriptor = (const float*)reader.ptr;
        CV_NEXT_SEQ_ELEM( kreader.seq->elem_size, kreader );
        CV_NEXT_SEQ_ELEM( reader.seq->elem_size, reader );
        int nearest_neighbor = naiveNearestNeighbor( descriptor, kp->laplacian, imageKeypoints, imageDescriptors );
        if( nearest_neighbor >= 0 )
        {
            ptpairs.push_back(i);
            ptpairs.push_back(nearest_neighbor);
        }
    }
}


geometry_msgs::Vector3 calculateChangeInPosition()
{
	sensor_msgs::CvBridge bridge;
	IplImage* leftImg = leftImages.front(); //bridge.imgMsgToCv(leftImages.front());
	IplImage* rightImg = rightImages.front(); //bridge.imgMsgToCv(rightImages.front());

	CvSeq* leftKeypoints;
	CvSeq* leftDescriptors;
	CvSeq* rightKeypoints;
	CvSeq* rightDescriptors;

	CvMemStorage* storage = cvCreateMemStorage(0);
	CvSURFParams params = cvSURFParams(500, 1);

	vector<int> ptpairs;
	vector<float> disparities;
	vector<float> distances;

	// Extract SURF keypoints/descriptors from both images
	cvExtractSURF(leftImg, 0, &leftKeypoints, &leftDescriptors, storage, params);
	cvExtractSURF(rightImg, 0, &rightKeypoints, &rightDescriptors, storage, params);

	// Find matching pairs between images
	findPairs(leftKeypoints, leftDescriptors, rightKeypoints, rightDescriptors, ptpairs);

	// Calculate horizontal offsets for each pair to use in distance calculation
	for (int i = 0; i < (int)ptpairs.size(); i += 2)
	{
		CvSURFPoint* r1 = (CvSURFPoint*)cvGetSeqElem( leftKeypoints, ptpairs[i] );
		CvSURFPoint* r2 = (CvSURFPoint*)cvGetSeqElem( rightKeypoints, ptpairs[i+1] );
		// TODO:  Fix the line below.  Don't think we can use pixel distances without scaling it somehow
		float disparity = fabs(r1->pt.x - r2->pt.x);
		disparities.push_back(disparity);
		distances.push_back(FOCAL_LENGTH * CAMERA_DISTANCE / disparity);
	}

	// TODO: Calculate optical flow vectors of features b/w timesteps
	// TODO: Publish only the change in position

	float dx = 0.0;	// TODO: insert real calculation here.  Need heading and time data
	float dy = 0.0;	// insert real calculation here
	
	return geometry_msgs::Vector3(dx, dy, 0.0);

	//currentPosition.x = dx;
	//currentPosition.y = dy;

	// Publish an estimate of position given optical flow vectors
	//svPub.publish(currentPosition);	

}


void processLeftImage(const sensor_msgs::ImageConstPtr &img)
{

	// TODO:  Fix how we acquire images from both cameras.  Code below won't work
	/*
	if (leftImages.size() >= 2)
	{
		leftImages.push(bridge.imgMsgToCv(img));
		leftImages.pop();
		calculatePosition();
	}
	else
	{
		leftImages.push(bridge.imgMsgToCv(img));
	}
	*/
}


void processRightImage(const sensor_msgs::ImageConstPtr &img)
{
	/*
	if (rightImages.size() >= 2)
	{
		rightImages.push(bridge.imgMsgToCv(img));
		rightImages.pop();
		calculatePosition();
	}
	else
	{
		rightImages.push(bridge.imgMsgToCv(img));
	}
	*/
}


/*void resetPosition(geometry_msgs::Point &position)
{
	currentPosition = position;
}*/


int main(int argc, char** argv)
{
	ros::init(argc, argv, "stereo_vision");
	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);

	// Initialize our current position to origin
	//currentPosition.x = 0.0;
	//currentPosition.y = 0.0;
	//currentPosition.z = 0.0;

	// Register service to reset current position
//	ros::ServiceServer service = nh.advertiseService("reset_position", resetPosition);

	// Register publisher for the center of the bin position in an image
	svPub = nh.advertise<geometry_msgs::Vector3>("odom_prim", 1);

	// Register publisher for image with bins highlighted
	//svImagePub = it.advertise("stereo_image", 1);

	// Subscribe to an image topic
	image_transport::Subscriber sub1 = it.subscribe("image_left", 1, processLeftImage);
	image_transport::Subscriber sub2 = it.subscribe("image_right", 1, processRightImage);

	ros::spin();
}

