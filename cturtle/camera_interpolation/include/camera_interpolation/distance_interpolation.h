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

class DistanceInterpolation
{
public:

	typedef double _DistanceType;
	typedef sensor_msgs::CameraInfo _CameraInfoType;

	/* constructor params */
	_CameraInfoType camera_info_;
	_DistanceType pixel_meters_;

	// get the pixel-meter calibration for a camera given the distance to an object, the observed pixel width of the object at that distance, and the diameter of the object in meters
	static _DistanceType getPixelMeters( _DistanceType distance_in_meters, int diameter_in_pixels, _DistanceType diameter_in_meters = 1.0 )
	{
		diameter_in_pixels * distance_in_meters / diameter_in_meters;
	}

	DistanceInterpolation()
	{
		//
	}

	DistanceInterpolation( _CameraInfoType camera_info, _DistanceType pixel_meters ) : camera_info_( camera_info ), pixel_meters_( pixel_meters )
	{
		//
	}

	// get the distance to an object given its diameter in meters and observed diameter in pixels
	_DistanceType distanceToFeature( int pixel_diameter, _DistanceType meter_diameter )
	{
		meter_diameter * pixel_meters_ / pixel_diameter;
	}
};

#endif /* DISTANCE_INTERPOLATION_H_ */
