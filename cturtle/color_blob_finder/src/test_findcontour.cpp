/*******************************************************************************
 *      test_findcontour
 * 
 *      Copyright (c) 2011, noah
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
 *      * Neither the name of "seabee3-ros-pkg" nor the names of its
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

#include <stdio.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <iostream>

int main( int argc, char* argv[] )
{
	if ( argc != 3 )
	{
		std::cerr << "Usage: " << argv[0] << " buoy_template_file video_input_file or " << argv[0] << " buoy_template_file \"camera\"" << std::endl;
		return -1;
	}
	int thresh1 = 200;
	int thresh2 = 100;
	int matchThresh = 1;
	cv::Moments moments;
	//cv::HuMoments humoments;
	cv::namedWindow( "Canny", CV_WINDOW_AUTOSIZE );
	cv::namedWindow( "Input", CV_WINDOW_AUTOSIZE );
	cv::createTrackbar( "Thresh1", "Canny", &thresh1, 255 );
	cv::createTrackbar( "Thresh2", "Canny", &thresh2, 255 );
	cv::createTrackbar( "MatchThresh", "Input", &matchThresh, 200 );

	std::string templateImageName( argv[1] );
	std::string videoName( argv[2] );

	cv::Mat templateImageInput = cv::imread( templateImageName, 1 );
	if ( templateImageInput.empty() )
	{
		std::cerr << "Could not open template file: " << templateImageName << std::endl;
	}
	cv::Mat templateImage;
	cv::cvtColor( templateImageInput, templateImage, CV_RGB2GRAY );
	cv::threshold( templateImage, templateImage, 10, 255, CV_THRESH_BINARY );

	std::vector<std::vector<cv::Point> > templatecontours;
	cv::findContours( templateImage, templatecontours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE );
	cv::Mat dispImage = templateImageInput;
	cv::drawContours( dispImage, templatecontours, -1, cv::Scalar( 255, 0, 0 ), 5 );

	std::vector<cv::Point> templateContour = templatecontours[0];

	cv::VideoCapture cap;
	if ( videoName == "camera" ) cap.open( 0 );
	else cap.open( videoName );
	if ( !cap.isOpened() )
	{
		std::cerr << "Could not open video" << std::endl;
		return -1;
	}
	while ( 1 )
	{
		cv::Mat inputFrame;
		cap >> inputFrame;

		cv::Mat bwInput;
		cv::cvtColor( inputFrame, bwInput, CV_RGB2GRAY );
		cv::Mat cannyImage;
		cv::Canny( bwInput, cannyImage, thresh1, thresh2 );
		std::vector<std::vector<cv::Point> > contours;
		cv::findContours( cannyImage, contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE );

		std::cout << "found " << contours.size() << " contours" << std::endl;
		if ( contours.size() ) cv::drawContours( inputFrame, contours, -1, cv::Scalar( 0, 255, 0 ), 1 );

		for ( size_t i = 0; i < contours.size(); ++i )
		{
			double matchQuality = cv::matchShapes( cv::Mat( templateContour ), cv::Mat( contours[i] ), CV_CONTOURS_MATCH_I1, 0 );


			//First calculate object moments
			moments = cv::moments( templateImage, 0 );
			//Now calculate hu moments
			//cv::GetHuMoments( &moments, &humoments );

			std::cout << "match quality = " << matchQuality << std::endl;

			if ( matchQuality < matchThresh )
			{
				std::cout << "match! quality = " << matchQuality << std::endl;
				
				std::vector<std::vector<cv::Point> > c; // Stupid Opencv...
				c.push_back( contours[i] );
				cv::drawContours( inputFrame, c, -1, cv::Scalar( 255, 0, 0 ), 5 );
			}
		}

		cv::imshow( "Template", dispImage );
		cv::imshow( "Input", inputFrame );
		cv::imshow( "Canny", cannyImage );
		
		if ( videoName == "camera" ) cv::waitKey( 50 );
		else cv::waitKey();
	}

	return 0;
}
