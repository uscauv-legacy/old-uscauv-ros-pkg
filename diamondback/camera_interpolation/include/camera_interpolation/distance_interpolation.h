/*******************************************************************************
 *
 *      distance_interpolation
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

#ifndef DISTANCE_INTERPOLATION_H_
#define DISTANCE_INTERPOLATION_H_

#include <stdio.h>

class DistanceInterpolation
{
public:

	typedef double _MetricDistanceType;
	typedef double _PixelDistanceType;
	typedef double _IORType;

	/* constructor params */
	_MetricDistanceType pixel_meters_;
	_IORType calibration_medium_ior_;

	// get the pixel-meter calibration for a camera given the distance to an object, the observed pixel width of the object at that distance, and the diameter of the object in meters
	static _MetricDistanceType getPixelMeters( _PixelDistanceType pixels, _MetricDistanceType distance, _MetricDistanceType diameter = 1.0 )
	{
		if ( diameter <= 0 )
		{
			printf( "Invalid diameter: %f\n", diameter );
			return 0;
		}
		if ( pixels <= 0 )
		{
			printf( "Invalid number of pixels: %f\n", pixels );
			return 0;
		}
		if ( distance <= 0 )
		{
			printf( "Invalid distance: %f\n", distance );
			return 0;
		}
		return pixels * distance / diameter;
	}

	DistanceInterpolation() :
		pixel_meters_( 0 ), calibration_medium_ior_( 1.0 )
	{
		//
	}

	DistanceInterpolation( _MetricDistanceType pixel_meters, _IORType calibration_medium_ior = 1.0 ) :
		pixel_meters_( pixel_meters ), calibration_medium_ior_( calibration_medium_ior )
	{
		//
	}

	// get the distance to an object given its diameter in meters and observed diameter in pixels
	_MetricDistanceType distanceToFeature( _PixelDistanceType pixels, _MetricDistanceType diameter, _IORType current_medium_ior = 1.0 )
	{
		return diameter * pixel_meters_ * current_medium_ior / ( calibration_medium_ior_ * _IORType( pixels ) );
	}
};

#endif /* DISTANCE_INTERPOLATION_H_ */
