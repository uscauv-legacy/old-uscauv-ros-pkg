/*******************************************************************************
 *
 *      color_segmenter
 *
 *      Copyright (c) 2010
 *
 *      Mike Gerow
 *
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

#include <base_image_proc/base_image_proc.h>
#include <color_defs/colors.h>
#include <color_segmenter/FindBlobs.h>
//#include <ros/console.h>
#include "color_segmenter.h"
#include <deque>
#include <vector>

typedef BaseImageProc<BaseNodeTypes::_DefaultReconfigureType, color_segmenter::FindBlobs> _BaseImageProc;

class ColorSegmenter: public _BaseImageProc
{
public:
	ColorSegmenter( ros::NodeHandle & nh ) :
		_BaseImageProc ( nh )
	{
		initCfgParams();

	}

	cv::Mat processImage( IplImage * ipl_img, _ServiceRequest & req, _ServiceResponse & resp )
	{
		//req.blob_descriptor.min_mass
		//req.blob_descriptor.color will be some number in ColorIds::*

		//color_segmenter::ColorBlob blob;
		//resp.blob_array.color_blobs.push_back( blob );

		//Use dynamic reconfigure for debug.  That'd be super fancy.
		bool debug = true;
		bool use_cv_flood_fill = false;
		//DYNAMIC RECONFIGURE

		cv_img_ = cv::Mat( ipl_img );
		//makeImageBinary(cv_img_, req.blob_descriptor.color);
		cv::Vec3b color = getColorVector(req.blob_descriptor.color);
		if (use_cv_flood_fill)
		{
			resp.blob_array.color_blobs = floodFillMethod(cv_img_, req.blob_descriptor.min_mass);
		}
		else
		{
			resp.blob_array.color_blobs = blobFindingMethod(cv_img_, req.blob_descriptor.min_mass, color);
		}

		if (debug)
		{
			drawSegments(cv_img_, resp.blob_array.color_blobs);
		}
		return cv_img_;
	}

};

//void makeImageBinary(cv::Mat & cv_img_, int color_int)
//{
//	cv::Vec3b color;
//	switch (color_int)
//	{
//	case ColorIds::red:
//		color = OutputColorRGB::red;
//		break;
//	case ColorIds::orange:
//		color = OutputColorRGB::orange;
//		break;
//	case ColorIds::yellow:
//		color = OutputColorRGB::yellow;
//		break;
//	case ColorIds::green:
//		color = OutputColorRGB::green;
//		break;
//	case ColorIds::blue:
//		color = OutputColorRGB::blue;
//		break;
//	case ColorIds::black:
//		color = OutputColorRGB::black;
//		break;
//	case ColorIds::white:
//		color = OutputColorRGB::white;
//		break;
//	case ColorIds::unknown:
//		color = OutputColorRGB::unknown;
//		break;
//	}
	//
	//DO SOMETHING FANCY HERE TO MAKE THE IMAGE TWO COLORS
//
//	return;
//}

cv::Vec3b getColorVector(int color_int)
{
	cv::Vec3b color;
	switch (color_int)
	{
	case ColorIds::red:
		color = OutputColorRGB::red;
		break;
	case ColorIds::orange:
		color = OutputColorRGB::orange;
		break;
	case ColorIds::yellow:
		color = OutputColorRGB::yellow;
		break;
	case ColorIds::green:
		color = OutputColorRGB::green;
		break;
	case ColorIds::blue:
		color = OutputColorRGB::blue;
		break;
	case ColorIds::black:
		color = OutputColorRGB::black;
		break;
	case ColorIds::white:
		color = OutputColorRGB::white;
		break;
	case ColorIds::unknown:
		color = OutputColorRGB::unknown;
		break;
	}
	return color;
}

color_segmenter::ColorBlob findBlob(cv::Mat & cv_img_, std::vector<std::vector<bool> > & table, int initx, int inity, bool include_diagonals, cv::Vec3b & color)
{
	color_segmenter::ColorBlob blob;
	std::deque<cv::Point> d;
	cv::Point curpoint;
	int modx, mody;
	long totx, toty;
	int pixel_shifts = 4;

	d.push_back(cv::Point(initx, inity));
	blob.mass += 1;
	totx = initx;
	toty = inity;
	while (d.size() != 0)
	{
		curpoint = d.front();
		d.pop_front();
		if (include_diagonals)
		{
			pixel_shifts = 8;
		}
		for (int i = 0; i < pixel_shifts; i++){
			switch (i)
			{
			case 0:
				modx = 1;
				mody = 0;
				break;
			case 1:
				modx = -1;
				mody = 0;
				break;
			case 2:
				modx = 0;
				mody = 1;
				break;
			case 3:
				modx = 0;
				mody = -1;
				break;
			case 4:
				modx = 1;
				mody = 1;
				break;
			case 5:
				modx = -1;
				mody = -1;
				break;
			case 6:
				modx = -1;
				mody = 1;
				break;
			case 7:
				modx = 1;
				mody = -1;
				break;
			}
			if (curpoint.x + modx >= 0
					&& curpoint.y + mody >= 0
					&& curpoint.x + modx <= cv_img_.rows
					&& curpoint.y + mody <= cv_img_.cols
					&& table[curpoint.x + modx][curpoint.y + mody]
			        && getBinPixelValue(cv_img_, color, curpoint.x + modx, curpoint.y + mody))
			{
				table[curpoint.x + modx][curpoint.y + mody] = false;
				blob.mass += 1;
				totx += curpoint.x + modx;
				toty += curpoint.y + mody;
				d.push_back(cv::Point(curpoint.x + modx, curpoint.y + mody));
			}
		}
	}
	blob.x = static_cast<double>(totx)/blob.mass;
	blob.y = static_cast<double>(toty)/blob.mass;
	return blob;
}

std::vector<color_segmenter::ColorBlob> blobFindingMethod(cv::Mat & cv_img_, int min_mass, cv::Vec3b & color)
{
	return blobFindingMethod(cv_img_, min_mass, color, false);
}

std::vector<color_segmenter::ColorBlob> blobFindingMethod(cv::Mat & cv_img_, int min_mass, cv::Vec3b & color, bool include_diagonals)
{
	std::vector<color_segmenter::ColorBlob> blob_vec;
	std::vector<std::vector<bool> > table;
	std::vector<bool> horizvec;
	color_segmenter::ColorBlob blob;

	//Build a 2d vector of boolean values to mark which pixels have yet to be searched
	for (int i = 0; i < cv_img_.cols; i++)
	{
		horizvec.push_back(true);
	}
	for (int i = 0; i < cv_img_.rows; i++)
	{
		table.push_back(horizvec);
	}

	for (int x = 0; x < cv_img_.rows; x++)
	{
		for (int y = 0; y < cv_img_.cols; y++)
		{
			if (table[x][y])
			{
				table[x][y] = false;
				if(getBinPixelValue(cv_img_, color, x, y) == true)
				{
					blob = findBlob(cv_img_, table, x, y, include_diagonals, color);
					if (blob.mass >= min_mass)
					{
						blob_vec.push_back(blob);
						ROS_INFO("Found blob at (%lf, %lf) with a mass of %lf.\n", blob.x, blob.y, blob.mass);
					}
				}
			}
		}
	}
	return blob_vec;
}

std::vector<color_segmenter::ColorBlob> floodFillMethod(cv::Mat & cv_img_, int min_mass)
{
	//Implement cv::floodFill to
	std::vector<color_segmenter::ColorBlob> color_blobs;

	return color_blobs;
}

bool getBinPixelValue(cv::Mat & cv_img_, cv::Vec3b & color, int x, int y)
{
	CvScalar s;
	s = cvGet2D(&cv_img_, x, y);
	if(color[0] == s.val[0] && color[1] == s.val[1] && color[2] == s.val[2])
	{
		return true;
	}
	return false;
}

void drawSegments(cv::Mat & cv_img_, std::vector<color_segmenter::ColorBlob> & color_blobs)
{
	for (unsigned int i = 0; i < color_blobs.size(); i++)
	{
		cv::circle(cv_img_, cv::Point(color_blobs[i].x, color_blobs[i].y), 2, cv::Scalar(0, 0, 255));
		cv::putText(cv_img_, weightString(i, color_blobs[i].mass), cv::Point(color_blobs[i].x, color_blobs[i].y), 0, 1, cv::Scalar(255, 0, 0));
	}
	return;
}

std::string weightString(int i, double weight)
{
	std::stringstream s;
	s << "Segment " << i << ". Weight: " << weight;
	return s.str();
}

int main( int argc, char **argv )
{
	ros::init( argc, argv, "color_segmenter" );
	ros::NodeHandle nh;

	ColorSegmenter color_segmenter( nh );
	color_segmenter.spin();

	return 0;
}
