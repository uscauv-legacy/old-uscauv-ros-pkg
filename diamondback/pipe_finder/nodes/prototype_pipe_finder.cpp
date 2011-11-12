/***************************************************************************
 *  nodes/prototype_pipe_finder.cpp
 *  --------------------
 *
 *  Copyright (c) 2011, Kathryn Yu
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are
 *  met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following disclaimer
 *    in the documentation and/or other materials provided with the
 *    distribution.
 *  * Neither the name of pipe_finder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 **************************************************************************/

#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>
#include <cmath>
#include <pipe_finder/pipe_finder.h>

using namespace cv;
using contour_types::_Contour;
using contour_types::_Point;

int main(int argc, char* argv[])
{
    if(argc != 2)
    {
      std::cerr << "Usage: " << argv[0] << " FILENAME" << std::endl;
      return -1;
    } //Must pass an image in

IplImage * inputimage = &IplImage( imread(argv[1]) );

//number of slices to divide the contours into
const unsigned int number_of_slices = 8;

//holds the vector of contours
std::vector<_Contour> contours;
//holds the contour in the above vector we care about
_Contour largest_blob;
//finds the centroid of the contour we care about
_Point the_centroid;
//makes the histogram we want
_Histogram final_histogram;

//use window to verify we got a good blob for checking
namedWindow("blob_to_check",1);

	adoptedProcessImage( inputimage,
						 contours );

	imshow("blob_to_check", cv::Mat( inputimage ));
	waitKey();

//use the perimeters of the contours to make sure we pick a good blob
unsigned int perimeter_to_check_by = 0, new_perimeter = 0;

for(auto contour = contours.begin(); contour != contours.end(); contour++)
{
	//if the perimeter is big enough
	if(validatePerimeter(inputimage, *contour))
	{
		//if the perimeter is larger than the last big blob we found
		if(new_perimeter > perimeter_to_check_by)
		{
			largest_blob = *contour;
			perimeter_to_check_by = new_perimeter;
		}
	}
}

if (perimeter_to_check_by == 0)
	{
		//if you can't find a good perimeter, weeeh
		std::cout<<"No valid blobs found."<<std::endl;
		return 0;
	}

	//otherwise, get centroid
	the_centroid = calculateCentroid( largest_blob.begin(),
									  largest_blob.end() );
	//so we can pass it here
	final_histogram = generateScaledOrderedHistogram( largest_blob,
													  the_centroid,
											  		  number_of_slices );
	std::cout<<"If we're here, it maybe worked!"<<std::endl;

	return 0;
}
