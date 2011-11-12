/***************************************************************************
 *  include/pipe_finder/pipe_finder.h
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

#ifndef PIPEFINDER_PIPEFINDER_PIPEFINDER_H_
#define PIPEFINDER_PIPEFINDER_PIPEFINDER_H_

#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>
#include <cmath>
#include <pipe_finder/histogram.h>
#include <pipe_finder/contour.h>

typedef Histogram<double> _Histogram;
using contour_types::_Contour;
using contour_types::_Point;

// calculate the average value of a list of points
cv::Point2d calculateCentroid( const _Contour::iterator & first_point, const _Contour::iterator & last_point )
{
	cv::Point2d average_point( 0, 0 ), pipe_centroid(0, 0);

	unsigned int num_points = 0;
	for( auto current_point = first_point; current_point != last_point; ++ current_point )
	{
		average_point.x += current_point->x;
		average_point.y += current_point->y;
		++num_points;
	}

	pipe_centroid.x = average_point.x / num_points;
	pipe_centroid.y = average_point.y / num_points;
	return pipe_centroid;
}

// partition the points around the centroid and build a histogram out of the distance from the average position
// of all the points in each slice to the centroid
// To implement: Want to integrate from first point in pie slice to last point instead
_Histogram calculateHistogram( _Contour contour,
							   _Point centroid,
							   unsigned int num_slices,
							   int longest_distance_index,
							   int longest_distance )
{
	// our histogram has one element per pie slice
	_Histogram histogram( num_slices );

	// our running average position
	_Point component_centroid;

	// keep track of the number of points in the given pie slice
	unsigned int slice_size = 0;

	// keep track of the current pie slice
	unsigned int slice_index = 0;

	//keep location of the longest distance
	longest_distance_index = 0;
	longest_distance = 0;

	// for( current_point : contour )
	for( auto current_point = contour.begin(); current_point != contour.end(); ++current_point )
	{
		const double angle_to_current_point = atan2( (double)( current_point->y - centroid.y ), (double)( current_point->x - centroid.x ) );
		const double angle_of_next_slice = slice_index * 360 / num_slices;

		if( angle_to_current_point >= angle_of_next_slice )
		{
			// we've found the start of a new slice, and therefore the end of the last slice; the current point is the divider
			// but we'll consider it to be part of the new slice
			// so the previous point must be the last point in the previous slice

			// divide the sum of the points in the slice by the number of points in the slice to get the average point
			// component_centroid /= slicesize
			component_centroid = _Point( ( component_centroid.x / slice_size ),
			                    ( component_centroid.y / slice_size ) );

			// calculate distance from component_centroid to contour_centroid
			//WANT TO INTEGRATE INSTEAD EDIT THIS
			double distance_to_centroid = sqrt( pow(component_centroid.x - centroid.x, 2) +
												pow(component_centroid.y - centroid.y, 2) );

			// keep track of longest distance
			if(distance_to_centroid > longest_distance)
			{
				longest_distance = distance_to_centroid;
				longest_distance_index = slice_index;
			}

			// add the bin for the last pie slice to the histogram
			histogram.push_back( distance_to_centroid );

			// as described above, the current point is now the first point of the new slice; start the running average again
			component_centroid = _Point( ( current_point->x - centroid.x ), ( current_point->y - centroid.y ) );

			// the new pie slice now has one value, so update its size accordingly
			slice_size = 1;

			// we've now moved on to the next pie slice
			slice_index ++;
		}
		else
		{
			// maintain the running sum of points
			component_centroid.x += current_point->x - centroid.x ;
			component_centroid.y += current_point->y - centroid.y ;


			// maintain the number of points in the current slice
			slice_size ++;
		}
	}
		return histogram;
}

// scale every element in the histogram equally such that the largest element is 1.0
void normalizeHistogram( _Histogram &histogram )
{
	double histogram_scale = 0;

	//take longest distance and find constant to make it 1.0
	histogram.updateStatistics( true );
	histogram_scale = 1/ ( histogram.max_ );

	for( auto current_bin = histogram.begin(); current_bin != histogram.end(); ++current_bin )
	{
		// scale current element
		*current_bin *= histogram_scale ;
	}
}

// shift the histogram such that the first bin has the largest value
// pass the longest_distance_location from the last function to this function
_Histogram shiftHistogram( _Histogram histogram )
{
	_Histogram shifted_histogram;

	histogram.updateStatistics( true );
	auto longest_distance_index = histogram.max_index_;

	//push all elements from the longest element to the end into the histogram
	for(auto i = histogram.begin() + longest_distance_index; i != histogram.end(); i++)
	{
		shifted_histogram.push_back(*i);
	}
	//Wraparound. Take remaining elements from the histogram's beginning to but not including the largest element
	for(auto j = histogram.begin(); j != histogram.begin() + longest_distance_index; j++)
	{
		shifted_histogram.push_back(*j);
	}

	return shifted_histogram;
}

// compare candidate_hist to template_hist
// compare histogram to a stored rectangle histogram.
double compareHistogram( _Histogram template_hist, _Histogram candidate_hist )
{
	//store differences into vector????
	std::vector<unsigned int> delta;

	//compare differences between two histograms???
	for (unsigned int i = 0; i < template_hist.size(); ++i)
	{
		delta.push_back( abs( template_hist[i] - candidate_hist[i] ) );
	}
	//pffffffffffft idk man
	return 0;
}

_Histogram generateScaledOrderedHistogram( _Contour contour, _Point some_centroid, unsigned int num_slices)
{
	//a nice clean packaged function to make the histogram (to use in main)
	_Histogram final_histogram;
	int longest_distance = 0;
	int longest_distance_index = 0;

	final_histogram = calculateHistogram( contour, some_centroid, num_slices, longest_distance_index, longest_distance );
	normalizeHistogram( final_histogram );
	final_histogram = shiftHistogram( final_histogram );

	return final_histogram;
}

bool validatePerimeter( IplImage * input_image, _Contour contour )
{
	//check a perimeter to see if it's large enough to care about
	const double perimScale = 10;
	double perimeter_bound = ( input_image->width + input_image->height ) / perimScale;
	double total_perimeter = cvContourPerimeter( &contour[0] );

	if ( total_perimeter >= perimeter_bound ) return true;
	return false;
}

void adoptedProcessImage( const IplImage * ipl_image, std::vector<_Contour> & contours )
{
	//Shamelesly ripped off from Ed (is this even ok)

				/*output_image = cvCreateImage( cvSize( ipl_image->width,
													   ipl_image->height ),
											 IPL_DEPTH_8U,
											 3 );*/

                cv::Mat current_image_mat( ipl_image );
                contour::findContours( current_image_mat, contours );
                // b, g, r
                /*const CvScalar red = cvScalar( 0, 0, 255 );
                cv::Mat output_image_mat( output_image_ );
                cv::drawContours( output_image_mat,
                                  contours,
                                  -1,
                                  red,
                                  1 );*/
}

#endif // PIPEFINDER_PIPEFINDER_PIPEFINDER_H_
