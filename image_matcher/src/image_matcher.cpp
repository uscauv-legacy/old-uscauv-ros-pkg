// Some of this code comes from the find_obj.cpp example that is included with OpenCV
// compareSURFDescriptors, naiveNearestNeighbor, and findPairs

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_matcher/MatchImage.h>
#include <cv_bridge/CvBridge.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <image_transport/image_transport.h>
#include <yaml-cpp/yaml.h>
#include <vector>
#include <fstream>
#include <time.h>

#include <boost/thread/thread.hpp> 
#include <boost/thread/mutex.hpp>

#define HESSIAN_THRESHOLD 500
#define EXTENDED_DESCRIPTORS 1

using namespace std;

// File names for the bin objects
string axeFile;
string clippersFile;
string hammerFile;
string macheteFile;

sensor_msgs::ImageConstPtr itsCurrentImage;
sensor_msgs::ImageConstPtr testImage;
CvMemStorage* objectStorage;
CvSeq* axeKeypoints;
CvSeq* axeDescriptors;
CvSeq* clippersKeypoints;
CvSeq* clippersDescriptors;
CvSeq* hammerKeypoints;
CvSeq* hammerDescriptors;
CvSeq* macheteKeypoints;
CvSeq* macheteDescriptors;
bool goodToGo = false;

boost::mutex image_mutex;

//int complete = 0; Delete this eventually

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

	// Delete this eventually if (total_cost == 0.0)	complete++;
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
	if ( dist1 < 0.6 * dist2 )
		return neighbor;
	return -1;
}


void findPairs( const CvSeq* objectKeypoints, const CvSeq* objectDescriptors,
           const CvSeq* imageKeypoints, const CvSeq* imageDescriptors, vector<int>& ptpairs )
{
	// Create structures to help us iterate through keypoints/descriptors
	CvSeqReader reader, kreader;
	cvStartReadSeq( objectKeypoints, &kreader );
	cvStartReadSeq( objectDescriptors, &reader );
	ptpairs.clear();

	for(int i = 0; i < objectDescriptors->total; i++ )
	{
		// Advance through object keypoints/descriptors
		const CvSURFPoint* kp = (const CvSURFPoint*)kreader.ptr;
		const float* descriptor = (const float*)reader.ptr;
		CV_NEXT_SEQ_ELEM( kreader.seq->elem_size, kreader );
		CV_NEXT_SEQ_ELEM( reader.seq->elem_size, reader );

		// Find the nearest descriptor/keypoint match in the image
		int nearest_neighbor = naiveNearestNeighbor( descriptor, kp->laplacian, imageKeypoints, imageDescriptors );
		if( nearest_neighbor >= 0 )
		{
			ptpairs.push_back(i);
			ptpairs.push_back(nearest_neighbor);
		}
	}
	//cout << "Total match:  " << complete << endl; Delete this eventually
}


// Save the most recent image we received.
// When our service is called then we process the image.
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  ROS_INFO("Image received");
  boost::lock_guard<boost::mutex> lock(image_mutex); 
  itsCurrentImage = msg;
  goodToGo = true;
}


bool MatchImageCallback(image_matcher::MatchImage::Request &req, image_matcher::MatchImage::Response &res)
{
  boost::lock_guard<boost::mutex> lock(image_mutex); 
  
        // Check to make sure we've received a valid image
	if (!goodToGo)	return false;

	sensor_msgs::CvBridge bridge;
	CvSeq* imageKeypoints;
	CvSeq* imageDescriptors;
	CvMemStorage* storage = cvCreateMemStorage(0);
	CvSURFParams params = cvSURFParams(HESSIAN_THRESHOLD, EXTENDED_DESCRIPTORS);

	// Convert image from sensor_msg::Image to IplImage
	IplImage* image = bridge.imgMsgToCv(itsCurrentImage);

	// Convert color image to grayscale
	IplImage* grayImage = cvCreateImage(cvGetSize(image), 8, 1);
	cvCvtColor(image, grayImage, CV_BGR2GRAY);

	// Extract SURF keypoints/descriptors from the image
	ROS_INFO("Extracting SURF keypoints/descriptors from image");
	cvExtractSURF(grayImage, 0, &imageKeypoints, &imageDescriptors, storage, params);

	// Find matching pairs between objects and image
	vector<int> matchingPairs;	// Holds matching keypoints b/w object and image
	ROS_INFO("Comparing keypoints of objects and local image");

	findPairs(axeKeypoints, axeDescriptors, imageKeypoints, imageDescriptors, matchingPairs);
	float axeProbability = 100 * (float)(matchingPairs.size()/2) / (float)axeDescriptors->total;

	matchingPairs.clear();
	findPairs(clippersKeypoints, clippersDescriptors, imageKeypoints, imageDescriptors, matchingPairs);
	float clippersProbability = 100 * (float)(matchingPairs.size()/2) / (float)clippersDescriptors->total;

	matchingPairs.clear();
	findPairs(hammerKeypoints, hammerDescriptors, imageKeypoints, imageDescriptors, matchingPairs);
	float hammerProbability = 100 * (float)(matchingPairs.size()/2) / (float)hammerDescriptors->total;

	matchingPairs.clear();
	findPairs(macheteKeypoints, macheteDescriptors, imageKeypoints, imageDescriptors, matchingPairs);
	float macheteProbability = 100 * (float)(matchingPairs.size()/2) / (float)macheteDescriptors->total;

	res.axe_probability = axeProbability;
	res.clippers_probability = clippersProbability;
	res.hammer_probability = hammerProbability;
	res.machete_probability = macheteProbability;	

	ROS_INFO("Probabilities --- Axe: %f Clippers: %f Hammer: %f Machete: %f", axeProbability, clippersProbability, hammerProbability, macheteProbability);

	return true;
}


// This is a competition specific function that loads the
// bin pictures in memory and extracts SURF keypoints/descriptors
void loadObjectImages()
{
	IplImage* axeImg;
	IplImage* clippersImg;
	IplImage* hammerImg;
	IplImage* macheteImg;
	CvMemStorage* objectStorage = cvCreateMemStorage(0);
	CvSURFParams params = cvSURFParams(HESSIAN_THRESHOLD, EXTENDED_DESCRIPTORS);

	// Extract images from their respective files
	axeImg = cvLoadImage(axeFile.c_str());
	clippersImg = cvLoadImage(clippersFile.c_str());
	hammerImg = cvLoadImage(hammerFile.c_str());
	macheteImg = cvLoadImage(macheteFile.c_str());

	// Create grayscale image templates.
	IplImage* grayAxe = cvCreateImage(cvGetSize(axeImg), 8, 1);
	IplImage* grayClippers = cvCreateImage(cvGetSize(clippersImg), 8, 1);
	IplImage* grayHammer = cvCreateImage(cvGetSize(hammerImg), 8, 1);
	IplImage* grayMachete = cvCreateImage(cvGetSize(macheteImg), 8, 1);

	// Convert all images to grayscale
	cvCvtColor(axeImg, grayAxe, CV_BGR2GRAY);
	cvCvtColor(clippersImg, grayClippers, CV_BGR2GRAY);
	cvCvtColor(hammerImg, grayHammer, CV_BGR2GRAY);
	cvCvtColor(macheteImg, grayMachete, CV_BGR2GRAY);

	// Extract all SURF keypoints and descriptors from grayscale images
	cvExtractSURF(grayAxe, 0, &axeKeypoints, &axeDescriptors, objectStorage, params);
	cvExtractSURF(grayClippers, 0, &clippersKeypoints, &clippersDescriptors, objectStorage, params);
	cvExtractSURF(grayHammer, 0, &hammerKeypoints, &hammerDescriptors, objectStorage, params);
	cvExtractSURF(grayMachete, 0, &macheteKeypoints, &macheteDescriptors, objectStorage, params);
}


int main(int argc, char** argv)
{
	// Initialize ros structures
	ros::init(argc, argv, "image_matcher");
	ros::NodeHandle nh;//("~");
	image_transport::ImageTransport it(nh);
	std::string transport;

	// File paths for objects
	//nh.param("image_transport", transport, std::string("raw"));
	nh.param("axe", axeFile, std::string("axe.png"));
	nh.param("clippers", clippersFile, std::string("clippers.png"));
	nh.param("hammer", hammerFile, std::string("hammer.png"));
	nh.param("machete", macheteFile, std::string("machete.png"));


	// Load the competition silhouettes in memory and extract the SURF keypoints/descriptors
	// This needs to be loaded first before we start subscribing to images
	loadObjectImages();
	ROS_INFO("Finished loading competition object silhouettes");

	// Subscribe to image topic
	image_transport::Subscriber sub = it.subscribe("image", 1, &imageCallback);

	// Register service
	ros::ServiceServer image_match_srv = nh.advertiseService("MatchImage", MatchImageCallback);

	ros::spin();
}

