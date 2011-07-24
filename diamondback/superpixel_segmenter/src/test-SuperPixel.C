#include <cstddef>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <iostream>
#include <limits>
#include <cstdio>

#include "CVSuperPixel.H"

// ######################################################################
// Parameters
// ######################################################################
int sigma = 3;
int c = 500;
int min_size = 100;
int hMean = 0;
int sMean = 0;
int vMean = 0;
int hWeight = 0;
int sWeight = 0;
int vWeight = 0;
int threshold = 0;
int scale = 25;

// ######################################################################
void saveSettings( int val, void* data )
{
	std::cout << "saving data" << std::endl;

	cv::FileStorage colorfile( "colors.yaml", cv::FileStorage::WRITE );
	if ( colorfile.isOpened() )
	{

		colorfile << "Sigma " << sigma;
		colorfile << "C" << c;
		colorfile << "MinSize" << min_size;
		colorfile << "HueMean" << hMean;
		colorfile << "SatMean" << sMean;
		colorfile << "ValMean" << vMean;
		colorfile << "HueWeight" << hWeight;
		colorfile << "SatWeight" << sWeight;
		colorfile << "ValWeight" << vWeight;
		colorfile << "Threshold" << threshold;
		colorfile << "Scale" << scale;
		colorfile.release();
	}
	else
	{
		std::cerr << " COULD NOT WRITE TO SETTINGS FILE!" << std::endl;
	}
}
;

// ######################################################################
void loadSettings()
{
	cv::FileStorage colorfile( "colors.yaml", cv::FileStorage::READ );
	if ( colorfile.isOpened() )
	{
		std::cout << "Loading Defaults..." << std::endl;

		sigma = int( colorfile["Sigma"] );
		c = int( colorfile["C"] );
		min_size = int( colorfile["MinSize"] );
		hMean = int( colorfile["HueMean"] );
		hWeight = int( colorfile["HueWeight"] );
		sMean = int( colorfile["SatMean"] );
		sWeight = int( colorfile["SatWeight"] );
		vMean = int( colorfile["ValMean"] );
		vWeight = int( colorfile["ValWeight"] );
		threshold = int( colorfile["Threshold"] );
		scale = int( colorfile["Scale"] );
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
	cv::VideoCapture cap( 0 );
	std::cout << "Done" << std::endl;

	if ( !cap.isOpened() )
	{
		std::cerr << "Cannot open video source" << std::endl;
		return -1;
	}

	// Setup the controls
	int dummy;
	cv::namedWindow( "Debug", CV_WINDOW_AUTOSIZE );
	cv::namedWindow( "Distance", CV_WINDOW_AUTOSIZE );
	cv::createTrackbar( "Sigma", "Debug", &sigma, 20 );
	cv::createTrackbar( "Image Scale %", "Debug", &scale, 100 );
	cv::createTrackbar( "MinSize", "Debug", &min_size, 1000 );
	cv::createTrackbar( "C", "Debug", &c, 1000 );
	cv::createTrackbar( "Save", "Debug", &dummy, 1, &saveSettings );
	cv::createTrackbar( "H", "Distance", &hMean, 255 );
	cv::createTrackbar( "H Weight", "Distance", &hWeight, 255 );
	cv::createTrackbar( "S", "Distance", &sMean, 255 );
	cv::createTrackbar( "S Weight", "Distance", &sWeight, 255 );
	cv::createTrackbar( "V", "Distance", &vMean, 255 );
	cv::createTrackbar( "V Weight", "Distance", &vWeight, 255 );
	cv::createTrackbar( "Threshold", "Distance", &threshold, 255 );

	while ( 1 )
	{
		// Read an image from the camera
		cv::Mat inputImage;
		cap >> inputImage;

		// Rescale the image so we don't waste cpu power
		float normScale = float( scale ) / 100.0;
		cv::Mat smallImage;
		cv::resize( inputImage, smallImage, cv::Size( inputImage.size().width * normScale, inputImage.size().height * normScale ) );

		// Convert the RGB input image to Hue-Saturation-Value
		cv::Mat hsvImage( smallImage.size(), CV_8UC3 );
		cv::cvtColor( smallImage, hsvImage, CV_BGR2HSV );

		cv::Mat distanceImg( hsvImage.size(), CV_8UC3 );
		cv::MatConstIterator_<cv::Vec3b> in_it = hsvImage.begin<cv::Vec3b> (), in_end = hsvImage.end<cv::Vec3b> ();
		cv::MatIterator_<cv::Vec3b> dist_it = distanceImg.begin<cv::Vec3b> ();
		while ( in_it != in_end )
		{
			// Extract the H,S, and V channels from the input image
			const unsigned char input_hue = ( *in_it )[0];
			const unsigned char input_sat = ( *in_it )[1];
			const unsigned char input_val = ( *in_it )[2];

			// calculate an adaptive scale such that all weights have equal "priority" but the range of the image will stay within
			// [0,255]
			const float dynamic_scale = 3.0 / ( hWeight + sWeight + vWeight );

			// since hue is a color wheel value, the distance from the desired hue value can never be more than 180 degrees
			// so wrap the value around 255 / 2 and scale the result between 0 and 255
			float hdist = 2 * ( abs( hMean - input_hue ) % ( 255 / 2 ) );
			float sdist = abs( sMean - input_sat );
			float vdist = abs( vMean - input_val );

			// Calculate the weighted distance to our target color
			float dist = dynamic_scale * ( hdist * hWeight + sdist * sWeight + vdist * vWeight );

			// Clamp the distance between 0-255 because we have to stuff the value into a byte
			if ( dist > 255 ) dist = 255;
			else if ( dist < 0 ) dist = 0;

			// Round the distance, and stuff it into a byte
			const unsigned char bytedist = dist;

			// Set all 3 channels of the distance image to the byte distance ( the
			// superpixel algorithm is only set up to work with 3-channel images for
			// now).
			( *dist_it )[0] = bytedist;
			( *dist_it )[1] = bytedist;
			( *dist_it )[2] = bytedist;

			// Let's check out the next pixel
			in_it++;
			dist_it++;
		}

		// Segment the distance image using Felzenszwalb's efficient graph-based
		// segmentation algorithm: http://people.cs.uchicago.edu/~pff/papers/seg-ijcv.pdf
		std::vector<std::vector<cv::Point> > groups = SuperPixelSegment( distanceImg, sigma, c, min_size );

		// Create a debug image just for show.
		cv::Mat debugImage = SuperPixelDebugImage( groups, smallImage );

		// Inspect each of the groups produced by the segmentation algorithm
		for ( size_t grpIdx = 0; grpIdx < groups.size(); grpIdx++ )
		{
			double avgDist = 0;
			int minX = std::numeric_limits<int>::max();
			int maxX = std::numeric_limits<int>::min();
			int minY = std::numeric_limits<int>::max();
			int maxY = std::numeric_limits<int>::min();

			// Compute the average distance of the current group, as well as the bounding box
			for ( size_t pntIdx = 0; pntIdx < groups[grpIdx].size(); pntIdx++ )
			{
				avgDist += distanceImg.at<cv::Vec3b> ( groups[grpIdx][pntIdx] )[0];
				minX = std::min( minX, groups[grpIdx][pntIdx].x );
				maxX = std::max( maxX, groups[grpIdx][pntIdx].x );
				minY = std::min( minY, groups[grpIdx][pntIdx].y );
				maxY = std::max( maxY, groups[grpIdx][pntIdx].y );
			}
			avgDist /= groups[grpIdx].size();

			// If the average distance in color space of this group is below some
			// threshold, then it's a match! Let's call it a buoy and draw a box
			// around it.
			if ( avgDist < threshold )
			{
				cv::rectangle( inputImage, cv::Point( minX / normScale, minY / normScale ), cv::Point( maxX / normScale, maxY / normScale ), 255, 2 );
				char buff[255];
				sprintf( buff, "Buoy- Conf:%f Size: %zu", ( 255 - avgDist ) / 255.0 * 100.0, groups[grpIdx].size() );
				cv::putText( inputImage, std::string( buff ), cv::Point( minX / normScale, minY / normScale ), cv::FONT_HERSHEY_SIMPLEX, .5, 128, 2 );
			}
		}

		// Display all of our various debugging images
		cv::Mat bigDebugImage, bigDistanceImg;
		cv::resize( debugImage, bigDebugImage, cv::Size( debugImage.size().width / normScale, debugImage.size().height / normScale ) );
		cv::resize( distanceImg, bigDistanceImg, cv::Size( distanceImg.size().width / normScale, distanceImg.size().height / normScale ) );
		cv::imshow( "Debug", bigDebugImage );
		cv::imshow( "Distance", bigDistanceImg );
		cv::imshow( "Buoys", inputImage );
		cv::waitKey( 50 );
	}

	return 0;
}

