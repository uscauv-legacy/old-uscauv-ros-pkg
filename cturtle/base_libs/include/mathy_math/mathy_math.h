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

#ifndef MATHY_MATH_H_
#define MATHY_MATH_H_

#include <math.h>
#include <opencv/cv.h>
class MathyMath
{
public:
	template<typename _T>
	static _T angleDistRel( const _T & a1, const _T & a2 ); //return the distance D from @a1 to @a2 such that -180 < D < 180

	template<typename _T>
	static void normalizeAngle( _T & a ); //force @a into 0 <= @a <= 360

	template<typename _T>
	static void capValue( _T & target, const _T & magCap ); //cap @target between -@magCap and @magCap

	template<typename _T>
	static void capValue( _T & target, const _T & lowerCap, const _T & upperCap ); //cap @target between @lowerCap and @upperCap

	template<typename _T>
	static void capValueProp( _T & t1, _T & t2, const _T magCap ); //scale @t1 and @t2 down by the same amount such that -@magCap <= t1, t2 <= magCap

	static double degToRad( const double &deg );

	static double radToDeg( const double &rad );

	/*static cv::Point2d vectorTo(const cv::Point3d & p1, const cv::Point3d & p2)
	 {
	 const cv::Point2d p1_cpy (p1.x, p1.y);
	 const cv::Point2d p2_cpy (p2.x, p2.y);
	 return vectorTo(p1_cpy, p2_cpy);
	 }*/

	static cv::Point2d vectorTo( const cv::Point2d & p1, const cv::Point2d & p2 );

	//calculates the ratio of difference between a1 and a2 such that angleRatio(x, x+180) = 1 and angleRatio(x, x) = 0
	static double linearAngleDiffRatio( const double & a1, const double & a2 );

	static double trigAngleDiffRatio( const double & a1, const double & a2 );

	static double ProbabilityDistribution( const double & u, const double & s, const double & x );

	static void polarToEuler( double &a ); //degrees people. USE IT!

	static void eulerToPolar( double &a );
};

// static
template<typename _T>
_T MathyMath::angleDistRel( const _T & a1, const _T & a2 ) //return the distance D from @a1 to @a2 such that -180 < D < 180
{
	_T c = a1 - a2;

	const _T c_p_360( 360 );
	const _T c_p_180( 180 );
	const _T c_n_360( -360 );
	const _T c_n_180( -180 );
	const _T c_zero( 0 );


	//this line does the following:
	//if c < -180, subtract 360 from a2; else if c > 180, add 360 to a2; else add 0 to a2
	//return a2 - a1
	return ( a2 + ( c < c_n_180 ? c_n_360 : ( c > c_p_180 ? c_p_360 : c_zero ) ) ) - a1;
}

// static
template<typename _T>
void MathyMath::normalizeAngle( _T & a ) //force @a into 0 <= @a <= 360
{
	//const _T c_p_360 ( 360 );
	//const int c_p_i_360 = (int) c_p_360;
	const int c_a_int = (int) a;
	const _T c_a_int_val( c_a_int );
	const _T c_a_diff = a - c_a_int_val;

	const _T result( c_a_int % 360 );

	a = result + c_a_diff;
}

// static
template<typename _T>
void MathyMath::capValue( _T & target, const _T & magCap ) //cap @target between -@magCap and @magCap
{
	capValue( target, -magCap, magCap );
}

// static
template<typename _T>
void MathyMath::capValue( _T & target, const _T & lowerCap, const _T & upperCap ) //cap @target between @lowerCap and @upperCap
{
	target = target < lowerCap ? lowerCap : ( target > upperCap ? upperCap : target ); //teh 1337
}

// static
template<typename _T>
void MathyMath::capValueProp( _T & t1, _T & t2, const _T magCap ) //scale @t1 and @t2 down by the same amount such that -@magCap <= t1, t2 <= magCap
{
	const _T largest = fabs( t1 ) > fabs( t2 ) ? t1 : t2;

	if ( fabs( largest ) > fabs( magCap ) )
	{
		//_T diff = fabs(largest) - magCap;
		const double div = fabs( (double) largest ) / fabs( (double) magCap ); //explicit use of double for scaling only
		const double t1_cpy( t1 );
		const double t2_cpy( t2 );

		t1 = _T( round( t1_cpy / div ) );
		t2 = _T( round( t2_cpy / div ) );
	}
}

#endif /* MATHY_MATH_H_ */
