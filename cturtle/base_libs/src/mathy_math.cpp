/*******************************************************************************
 *
 *      mathy_math
 * 
 *      Copyright (c) 2010, Edward T. Kaszubski (ekaszubski@gmail.com)
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

#include <mathy_math/mathy_math.h>

// static
double MathyMath::degToRad( const double &deg )
{
	return deg * M_PI / 180.0;
}

// static
double MathyMath::radToDeg( const double &rad )
{
	return rad * 180.0 / M_PI;
}

/*static cv::Point2d vectorTo(const cv::Point3d & p1, const cv::Point3d & p2)
 {
 const cv::Point2d p1_cpy (p1.x, p1.y);
 const cv::Point2d p2_cpy (p2.x, p2.y);
 return vectorTo(p1_cpy, p2_cpy);
 }*/

// static
cv::Point2d MathyMath::vectorTo( const cv::Point2d & p1, const cv::Point2d & p2 )
{
	//cout << "->vectorTo" << endl;
	cv::Point2d p( p2.x - p1.x, p2.y - p1.y );
	double dist = sqrt( pow( p.x, 2 ) + pow( p.y, 2 ) );
	double angle = 0.0;
	if ( p.x == 0.0 )
	{
		if ( p.y >= 0.0 )
		{
			angle = 90.0;
		}
		else
		{
			angle = 270.0;
		}
	}
	else
	{
		angle = radToDeg( fabs( atan( p.y / p.x ) ) );
		if ( p.x >= 0.0 && p.y >= 0.0 )
		{
			//
		}
		else if ( p.x < 0.0 && p.y >= 0.0 )
		{
			angle = ( 90.0 - angle ) + 90.0;
		}
		else if ( p.x < 0.0 && p.y < 0.0 )
		{
			angle += 180.0;
		}
		else
		{
			angle = ( 90.0 - angle ) + 270.0;
		}
	}
	normalizeAngle( angle );
	//cout << "<-vectorTo" << endl;
	return cv::Point2d( dist, angle );
}

//calculates the ratio of difference between a1 and a2 such that angleRatio(x, x+180) = 1 and angleRatio(x, x) = 0
// static
double MathyMath::linearAngleDiffRatio( const double & a1, const double & a2 )
{
	//cout << "->angleDiffRatio" << endl;
	double theAngle = fabs( a1 - a2 );
	while ( theAngle > 360 )
	{
		theAngle -= 360;
	}
	while ( theAngle < 0 )
	{
		theAngle += 360;
	}
	if ( theAngle > 180 )
	{
		theAngle = 360.0 - theAngle;
	}
	theAngle /= 180.0;


	//cout << "<-angleDiffRatio" << endl;
	return theAngle;
}

// static
double MathyMath::trigAngleDiffRatio( const double & a1, const double & a2 )
{
	double d = sqrt( pow( cos( degToRad( a1 ) ) - cos( degToRad( a2 ) ), 2 ) + pow( sin( degToRad( a1 ) ) - sin( degToRad( a2 ) ), 2 ) );
	return d / 2.0;
}

// static
double MathyMath::ProbabilityDistribution( const double & u, const double & s, const double & x )
{
	return exp( -pow( x - u, 2.0 ) / ( 2.0 * pow( s, 2 ) ) ) / sqrt( 2.0 * M_PI * pow( s, 2.0 ) );
}

// static
void MathyMath::polarToEuler( double &a ) //degrees people. USE IT!
{
	a += -90 + ( a < 0.0 ? 360.0 : 0.0 );
}

//static
void MathyMath::eulerToPolar( double &a )
{
	a += 90 + ( a < 0.0 ? 360.0 : 0.0 );
}
