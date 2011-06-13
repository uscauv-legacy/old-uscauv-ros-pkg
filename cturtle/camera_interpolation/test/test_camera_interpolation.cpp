/*******************************************************************************
 *
 *      test_camera_interpolation
 * 
 *      Copyright (c) 2011, Edward T. Kaszubski ( ekaszubski@gmail.com )
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

#include <stdexcept> // for std::runtime_error
#include <gtest/gtest.h>
#include <camera_interpolation/core.h>
#include <math.h>

#define dist( x, y ) fabs( x - y )
#define EXPECT_LER( r, x ) EXPECT_LE( x, r )

TEST( DistanceInterpolation, initialization )
{
	DistanceInterpolation distance_interpolation;
	EXPECT_EQ( 1.0, distance_interpolation.calibration_medium_ior_ )<< "Calibration medium ior not initialized properly";

	distance_interpolation = DistanceInterpolation( DistanceInterpolation::getPixelMeters( 200, 1.0, 2.0 ), 1.0 );
	EXPECT_EQ( 1.0, distance_interpolation.calibration_medium_ior_ )<< "Calibration medium ior not initialized properly";

	distance_interpolation = DistanceInterpolation( DistanceInterpolation::getPixelMeters( 200, 1.0, 2.0 ), 1.33 );
	EXPECT_EQ( 1.33, distance_interpolation.calibration_medium_ior_ )<< "Calibration medium ior not initialized properly";
}

TEST( DistanceInterpolation, getPixelMeters )
{
	DistanceInterpolation::_PixelDistanceType pixels;
	DistanceInterpolation::_MetricDistanceType distance;
	DistanceInterpolation::_MetricDistanceType diameter;

	distance = 1.0;
	pixels = 100;
	EXPECT_LER( 0.01, dist( 100.0, DistanceInterpolation::getPixelMeters( pixels, distance ) ) )<< "pixel-meters calculation for [" << pixels << ", " << distance << ", " << diameter << "] failed";

	distance = 2.0;
	pixels = 100;
	EXPECT_LER( 0.01, dist( 200.0, DistanceInterpolation::getPixelMeters( pixels, distance ) ) )<< "pixel-meters calculation for [" << pixels << ", " << distance << ", " << diameter << "] failed";

	distance = 1.0;
	pixels = 100;
	diameter = 2.0;
	EXPECT_LER( 0.01, dist( 50.0, DistanceInterpolation::getPixelMeters( pixels, distance, diameter ) ) )<< "pixel-meters calculation for [" << pixels << ", " << distance << ", " << diameter << "] failed";

	distance = 2.0;
	pixels = 100;
	diameter = 2.0;
	EXPECT_LER( 0.01, dist( 100.0, DistanceInterpolation::getPixelMeters( pixels, distance, diameter ) ) )<< "pixel-meters calculation for [" << pixels << ", " << distance << ", " << diameter << "] failed";

	distance = 1.0;
	pixels = 100 * 1.33;
	diameter = 1.0;
	EXPECT_LER( 0.01, dist( 100.0 * 1.33, DistanceInterpolation::getPixelMeters( pixels, distance, diameter ) ) )<< "pixel-meters calculation for [" << pixels << ", " << distance << ", " << diameter << "] failed";
}

TEST( DistanceInterpolation, distanceToFeatureAirToAir )
{
	DistanceInterpolation distance_interpolation( DistanceInterpolation::getPixelMeters( 200, 1.0, 2.0 ), 1.0 );

	DistanceInterpolation::_PixelDistanceType pixels;
	DistanceInterpolation::_MetricDistanceType diameter;
	DistanceInterpolation::_IORType ior;

	pixels = 200;
	diameter = 2.0;
	ior = 1.0;
	double distance_air1 = distance_interpolation.distanceToFeature( pixels, diameter, ior );
	EXPECT_LER( 0.01, dist( 1.0, distance_air1 ) )<< "distance calculation failed for [" << pixels << ", " << diameter << ", " << ior << "] failed";

	pixels = 100;
	diameter = 2.0;
	ior = 1.0;
	double distance_air2 = distance_interpolation.distanceToFeature( pixels, diameter, ior );
	EXPECT_LER( 0.01, dist( 2.0, distance_air2 ) )<< "distance calculation failed for [" << pixels << ", " << diameter << ", " << ior << "] failed";

	pixels = 400;
	diameter = 2.0;
	ior = 1.0;
	double distance_air3 = distance_interpolation.distanceToFeature( pixels, diameter, ior );
	EXPECT_LER( 0.01, dist( 0.5, distance_air3 ) )<< "distance calculation failed for [" << pixels << ", " << diameter << ", " << ior << "] failed";

	pixels = 50;
	diameter = 2.0;
	ior = 1.0;
	double distance_air4 = distance_interpolation.distanceToFeature( pixels, diameter, ior );
	EXPECT_LER( 0.01, dist( 4.0, distance_air4 ) )<< "distance calculation failed for [" << pixels << ", " << diameter << ", " << ior << "] failed";

	pixels = 800;
	diameter = 2.0;
	ior = 1.0;
	double distance_air5 = distance_interpolation.distanceToFeature( pixels, diameter, ior );
	EXPECT_LER( 0.01, dist( 0.25, distance_air5 ) )<< "distance calculation failed for [" << pixels << ", " << diameter << ", " << ior << "] failed";

	pixels = 100;
	diameter = 1.0;
	ior = 1.0;
	double distance_air6 = distance_interpolation.distanceToFeature( pixels, diameter, ior );
	EXPECT_LER( 0.01, dist( 1.0, distance_air6 ) )<< "distance calculation failed for [" << pixels << ", " << diameter << ", " << ior << "] failed";

	EXPECT_LER( 0.01, dist( distance_air1, distance_air6 ) )<< "distance calculation for 1-meter calibration object and 2-meter calibration object aren't consistent";

	pixels = 50;
	diameter = 1.0;
	ior = 1.0;
	double distance_air7 = distance_interpolation.distanceToFeature( pixels, diameter, ior );
	EXPECT_LER( 0.01, dist( 2.0, distance_air7 ) )<< "distance calculation failed for [" << pixels << ", " << diameter << ", " << ior << "] failed";

	EXPECT_LER( 0.01, dist( distance_air2, distance_air7 ) )<< "distance calculation for 1-meter calibration object and 2-meter calibration object aren't consistent";

	pixels = 200;
	diameter = 1.0;
	ior = 1.0;
	double distance_air8 = distance_interpolation.distanceToFeature( pixels, diameter, ior );
	EXPECT_LER( 0.01, dist( 0.5, distance_air8 ) )<< "distance calculation failed for [" << pixels << ", " << diameter << ", " << ior << "] failed";

	EXPECT_LER( 0.01, dist( distance_air3, distance_air8 ) )<< "distance calculation for 1-meter calibration object and 2-meter calibration object aren't consistent";
}

TEST( DistanceInterpolation, distanceToFeatureWaterToWater )
{
	DistanceInterpolation distance_interpolation( DistanceInterpolation::getPixelMeters( 200 * 1.33, 1.0, 2.0 ), 1.33 );

	DistanceInterpolation::_PixelDistanceType pixels;
	DistanceInterpolation::_MetricDistanceType diameter;
	DistanceInterpolation::_IORType ior;

	pixels = 200 * 1.33;
	diameter = 2.0;
	ior = 1.33;
	double distance_water1 = distance_interpolation.distanceToFeature( pixels, diameter, ior );
	EXPECT_LER( 0.01, dist( 1.0, distance_water1 ) )<< "distance calculation failed for [" << pixels << ", " << diameter << ", " << ior << "] failed";

	pixels = 100 * 1.33;
	diameter = 2.0;
	ior = 1.33;
	double distance_water2 = distance_interpolation.distanceToFeature( pixels, diameter, ior );
	EXPECT_LER( 0.01, dist( 2.0, distance_water2 ) )<< "distance calculation failed for [" << pixels << ", " << diameter << ", " << ior << "] failed";

	pixels = 400 * 1.33;
	diameter = 2.0;
	ior = 1.33;
	double distance_water3 = distance_interpolation.distanceToFeature( pixels, diameter, ior );
	EXPECT_LER( 0.01, dist( 0.5, distance_water3 ) )<< "distance calculation failed for [" << pixels << ", " << diameter << ", " << ior << "] failed";

	pixels = 50 * 1.33;
	diameter = 2.0;
	ior = 1.33;
	double distance_water4 = distance_interpolation.distanceToFeature( pixels, diameter, ior );
	EXPECT_LER( 0.01, dist( 4.0, distance_water4 ) )<< "distance calculation failed for [" << pixels << ", " << diameter << ", " << ior << "] failed";

	pixels = 800 * 1.33;
	diameter = 2.0;
	ior = 1.33;
	double distance_water5 = distance_interpolation.distanceToFeature( pixels, diameter, ior );
	EXPECT_LER( 0.01, dist( 0.25, distance_water5 ) )<< "distance calculation failed for [" << pixels << ", " << diameter << ", " << ior << "] failed";

	pixels = 100 * 1.33;
	diameter = 1.0;
	ior = 1.33;
	double distance_water6 = distance_interpolation.distanceToFeature( pixels, diameter, ior );
	EXPECT_LER( 0.01, dist( 1.0, distance_water6 ) )<< "distance calculation failed for [" << pixels << ", " << diameter << ", " << ior << "] failed";

	EXPECT_LER( 0.01, dist( distance_water1, distance_water6 ) )<< "distance calculation for 1-meter calibration object and 2-meter calibration object aren't consistent";

	pixels = 50 * 1.33;
	diameter = 1.0;
	ior = 1.33;
	double distance_water7 = distance_interpolation.distanceToFeature( pixels, diameter, ior );
	EXPECT_LER( 0.01, dist( 2.0, distance_water7 ) )<< "distance calculation failed for [" << pixels << ", " << diameter << ", " << ior << "] failed";

	EXPECT_LER( 0.01, dist( distance_water2, distance_water7 ) )<< "distance calculation for 1-meter calibration object and 2-meter calibration object aren't consistent";

	pixels = 200 * 1.33;
	diameter = 1.0;
	ior = 1.33;
	double distance_water8 = distance_interpolation.distanceToFeature( pixels, diameter, ior );
	EXPECT_LER( 0.01, dist( 0.5, distance_water8 ) )<< "distance calculation failed for [" << pixels << ", " << diameter << ", " << ior << "] failed";

	EXPECT_LER( 0.01, dist( distance_water3, distance_water8 ) )<< "distance calculation for 1-meter calibration object and 2-meter calibration object aren't consistent";
}

TEST( DistanceInterpolation, distanceToFeatureAirToWater )
{
	DistanceInterpolation distance_interpolation( DistanceInterpolation::getPixelMeters( 200, 1.0, 2.0 ), 1.0 );

	DistanceInterpolation::_PixelDistanceType pixels;
	DistanceInterpolation::_MetricDistanceType diameter;
	DistanceInterpolation::_IORType ior;

	pixels = 200 * 1.33;
	diameter = 2.0;
	ior = 1.33;
	double distance_water1 = distance_interpolation.distanceToFeature( pixels, diameter, ior );
	EXPECT_LER( 0.01, dist( 1.0, distance_water1 ) )<< "distance calculation failed for [" << pixels << ", " << diameter << ", " << ior << "] failed";

	pixels = 100 * 1.33;
	diameter = 2.0;
	ior = 1.33;
	double distance_water2 = distance_interpolation.distanceToFeature( pixels, diameter, ior );
	EXPECT_LER( 0.01, dist( 2.0, distance_water2 ) )<< "distance calculation failed for [" << pixels << ", " << diameter << ", " << ior << "] failed";

	pixels = 400 * 1.33;
	diameter = 2.0;
	ior = 1.33;
	double distance_water3 = distance_interpolation.distanceToFeature( pixels, diameter, ior );
	EXPECT_LER( 0.01, dist( 0.5, distance_water3 ) )<< "distance calculation failed for [" << pixels << ", " << diameter << ", " << ior << "] failed";

	pixels = 50 * 1.33;
	diameter = 2.0;
	ior = 1.33;
	double distance_water4 = distance_interpolation.distanceToFeature( pixels, diameter, ior );
	EXPECT_LER( 0.01, dist( 4.0, distance_water4 ) )<< "distance calculation failed for [" << pixels << ", " << diameter << ", " << ior << "] failed";

	pixels = 800 * 1.33;
	diameter = 2.0;
	ior = 1.33;
	double distance_water5 = distance_interpolation.distanceToFeature( pixels, diameter, ior );
	EXPECT_LER( 0.01, dist( 0.25, distance_water5 ) )<< "distance calculation failed for [" << pixels << ", " << diameter << ", " << ior << "] failed";

	pixels = 100 * 1.33;
	diameter = 1.0;
	ior = 1.33;
	double distance_water6 = distance_interpolation.distanceToFeature( pixels, diameter, ior );
	EXPECT_LER( 0.01, dist( 1.0, distance_water6 ) )<< "distance calculation failed for [" << pixels << ", " << diameter << ", " << ior << "] failed";

	EXPECT_LER( 0.01, dist( distance_water1, distance_water6 ) )<< "distance calculation for 1-meter calibration object and 2-meter calibration object aren't consistent";

	pixels = 50 * 1.33;
	diameter = 1.0;
	ior = 1.33;
	double distance_water7 = distance_interpolation.distanceToFeature( pixels, diameter, ior );
	EXPECT_LER( 0.01, dist( 2.0, distance_water7 ) )<< "distance calculation failed for [" << pixels << ", " << diameter << ", " << ior << "] failed";

	EXPECT_LER( 0.01, dist( distance_water2, distance_water7 ) )<< "distance calculation for 1-meter calibration object and 2-meter calibration object aren't consistent";

	pixels = 200 * 1.33;
	diameter = 1.0;
	ior = 1.33;
	double distance_water8 = distance_interpolation.distanceToFeature( pixels, diameter, ior );
	EXPECT_LER( 0.01, dist( 0.5, distance_water8 ) )<< "distance calculation failed for [" << pixels << ", " << diameter << ", " << ior << "] failed";

	EXPECT_LER( 0.01, dist( distance_water3, distance_water8 ) )<< "distance calculation for 1-meter calibration object and 2-meter calibration object aren't consistent";
}

TEST( DistanceInterpolation, distanceToFeatureWaterToAir )
{
	DistanceInterpolation distance_interpolation( DistanceInterpolation::getPixelMeters( 200 * 1.33, 1.0, 2.0 ), 1.33 );

	DistanceInterpolation::_PixelDistanceType pixels;
	DistanceInterpolation::_MetricDistanceType diameter;
	DistanceInterpolation::_IORType ior;

	pixels = 200;
	diameter = 2.0;
	ior = 1.0;
	double distance_air1 = distance_interpolation.distanceToFeature( pixels, diameter, ior );
	EXPECT_LER( 0.01, dist( 1.0, distance_air1 ) )<< "distance calculation failed for [" << pixels << ", " << diameter << ", " << ior << "] failed";

	pixels = 100;
	diameter = 2.0;
	ior = 1.0;
	double distance_air2 = distance_interpolation.distanceToFeature( pixels, diameter, ior );
	EXPECT_LER( 0.01, dist( 2.0, distance_air2 ) )<< "distance calculation failed for [" << pixels << ", " << diameter << ", " << ior << "] failed";

	pixels = 400;
	diameter = 2.0;
	ior = 1.0;
	double distance_air3 = distance_interpolation.distanceToFeature( pixels, diameter, ior );
	EXPECT_LER( 0.01, dist( 0.5, distance_air3 ) )<< "distance calculation failed for [" << pixels << ", " << diameter << ", " << ior << "] failed";

	pixels = 50;
	diameter = 2.0;
	ior = 1.0;
	double distance_air4 = distance_interpolation.distanceToFeature( pixels, diameter, ior );
	EXPECT_LER( 0.01, dist( 4.0, distance_air4 ) )<< "distance calculation failed for [" << pixels << ", " << diameter << ", " << ior << "] failed";

	pixels = 800;
	diameter = 2.0;
	ior = 1.0;
	double distance_air5 = distance_interpolation.distanceToFeature( pixels, diameter, ior );
	EXPECT_LER( 0.01, dist( 0.25, distance_air5 ) )<< "distance calculation failed for [" << pixels << ", " << diameter << ", " << ior << "] failed";

	pixels = 100;
	diameter = 1.0;
	ior = 1.0;
	double distance_air6 = distance_interpolation.distanceToFeature( pixels, diameter, ior );
	EXPECT_LER( 0.01, dist( 1.0, distance_air6 ) )<< "distance calculation failed for [" << pixels << ", " << diameter << ", " << ior << "] failed";

	EXPECT_LER( 0.01, dist( distance_air1, distance_air6 ) )<< "distance calculation for 1-meter calibration object and 2-meter calibration object aren't consistent";

	pixels = 50;
	diameter = 1.0;
	ior = 1.0;
	double distance_air7 = distance_interpolation.distanceToFeature( pixels, diameter, ior );
	EXPECT_LER( 0.01, dist( 2.0, distance_air7 ) )<< "distance calculation failed for [" << pixels << ", " << diameter << ", " << ior << "] failed";

	EXPECT_LER( 0.01, dist( distance_air2, distance_air7 ) )<< "distance calculation for 1-meter calibration object and 2-meter calibration object aren't consistent";

	pixels = 200;
	diameter = 1.0;
	ior = 1.0;
	double distance_air8 = distance_interpolation.distanceToFeature( pixels, diameter, ior );
	EXPECT_LER( 0.01, dist( 0.5, distance_air8 ) )<< "distance calculation failed for [" << pixels << ", " << diameter << ", " << ior << "] failed";

	EXPECT_LER( 0.01, dist( distance_air3, distance_air8 ) )<< "distance calculation for 1-meter calibration object and 2-meter calibration object aren't consistent";
}

int main( int argc, char ** argv )
{
	testing::InitGoogleTest( &argc, argv );
	return RUN_ALL_TESTS();
}
