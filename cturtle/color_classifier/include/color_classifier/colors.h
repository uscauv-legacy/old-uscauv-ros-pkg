/*******************************************************************************
 *
 *      colors
 * 
 *      Copyright (c) 2010, edward
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

#ifndef COLORS_H_
#define COLORS_H_

#include <opencv/cv.h>

struct ColorIds
{
	const static int red = 0;
	const static int orange = 1;
	const static int yellow = 2;
	const static int green = 3;
	const static int blue = 4;
	const static int black = 5;
	const static int white = 6;
	const static int unknown = 7;
};

struct OutputColorRGB
{
	const static cv::Vec3b red, orange, yellow, green, blue, black, white, unknown;
};

template<class _pixel_type = uchar>
struct DataRange
{
	_pixel_type min, max;

	bool isInRange( _pixel_type value, bool data_wraps = false )
	{
		return ( value >= min && value < max ) || ( data_wraps && min > max && ( value >= min || value < max ) );
	}
};

template<class _pixel_type = uchar>
struct ColorRange
{
	ColorRange<_pixel_type> i, j, k;
};

struct ColorSpectrumHSV
{
	DataRange<double> red, orange, yellow, green, blue;
	double black_l_threshold, white_l_threshold, unknown_s_threshold;
};

#endif /* COLORS_H_ */
