/*******************************************************************************
 *
 *      math
 * 
 *      Copyright (c) 2011, edward
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
 *      * Neither the name of "interaction-ros-pkg" nor the names of its
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

#ifndef MATH_H_
#define MATH_H_

#include <math.h>
#include <stdlib.h>
#include <time.h>
#include <type_traits>
#include <limits>

namespace math_utils
{
	template<class _DataType>
	struct HasDecAccuracy
	{
		const static bool val = (_DataType) 0.5 > 0;
	};

	template<class _DataType = int>
	class Point
	{
	public:
		_DataType x, y;

		Point( _DataType x_ = _DataType(), _DataType y_ = _DataType() ) :
			x( x_ ), y( y_ )
		{
			//
		}
	};

	template<class _DataType>
	static _DataType degToRad( const _DataType & degrees )
	{
		const static _DataType conversion_ratio = _DataType( M_PI ) / _DataType( 180 );
		return degrees * conversion_ratio;
	}

	template<class _DataType>
	static _DataType radToDeg( const _DataType & radians )
	{
		const static _DataType conversion_ratio = _DataType( 180 ) / _DataType( M_PI );
		return radians * conversion_ratio;
	}

	// by default: seed the random number generator if we haven't already done so, then return a number between 0 and 1
	// if force_re_seed is used, the number generator is re-seeded
	// if prevent_re_seed is used, number generator is never re-seeded; note that this condition only applies on the first call of this function
	static double drand( bool force_re_seed = false, bool prevent_re_seed = false )
	{
		static bool seeded = prevent_re_seed;
		if ( !seeded || force_re_seed )
		{
			srand( time( NULL ) );
			seeded = true;
		}

		return (double) rand() / RAND_MAX;
	}

	// return a random number between "low" and "high"; if "nonzero" is true then this value will never be zero
	template<class _DataType>
	static _DataType rand( const _DataType & low, const _DataType & high_, const bool & nonzero = false )
	{
		const static bool dec_accuracy = HasDecAccuracy<_DataType>::val;
		const _DataType high = dec_accuracy ? high_ : high_ + 1;

		_DataType value;
		do
		{
			value = low + ( high - low ) * drand();
		}
		while ( nonzero && (_DataType) value == 0 && ! ( low == high && low == 0 ) );

		return (_DataType) value;
	}

	// _PointType must have publicly-accessible members "x" and "y"
	template<class _PointType>
	_PointType vectorTo( const _PointType & from_point, const _PointType & to_point )
	{
		return vectorTo( from_point, to_point, from_point.x );
	}

	// _PointType must have publicly-accessible members "x" and "y"
	// point_data is used here solely to deduce _PointDataType
	template<class _PointType, class _PointDataType>
	_PointType vectorTo( const _PointType & from_point, const _PointType & to_point, const _PointDataType point_data )
	{
		const static _PointDataType ZERO( 0 );
		const static _PointDataType POS_90( 90 );
		const static _PointDataType POS_270( 270 );
		const static _PointDataType POS_180( 180 );

		static _PointType vec;
		static _PointDataType dist;
		static _PointDataType angle;

		dist = sqrt( pow( vec.x, 2 ) + pow( vec.y, 2 ) );
		vec.x = to_point.x - from_point.x;
		vec.y = to_point.y - from_point.y;

		if ( vec.x == ZERO )
		{
			if ( vec.y >= ZERO )
			{
				angle = POS_90;
			}
			else
			{
				angle = POS_270;
			}
		}
		else
		{
			angle = radToDeg( fabs( atan( vec.y / vec.x ) ) );
			if ( vec.x >= ZERO && vec.y >= ZERO )
			{
				//
			}
			else if ( vec.x < ZERO && vec.y >= ZERO )
			{
				angle = ( POS_90 - angle ) + POS_90;
			}
			else if ( vec.x < ZERO && vec.y < ZERO )
			{
				angle += POS_180;
			}
			else
			{
				angle = ( POS_90 - angle ) + POS_270;
			}
		}
		normalizeAngle( angle );

		vec.x = dist;
		vec.y = angle;

		return vec;
	}

	//calculates the ratio of difference between a1 and a2 such that angleRatio(x, x+180) = 1 and angleRatio(x, x) = 0
	template<class _DataType>
	static _DataType linearAngleDiffRatio( const _DataType & from_angle, const _DataType & to_angle )
	{
		static _DataType result;
		const static _DataType POS_360( 360 );
		const static _DataType POS_180( 180 );
		const static _DataType ZERO( 0 );

		result = fabs( from_angle - to_angle );

		while ( result > POS_360 )
		{
			result -= POS_360;
		}
		while ( result < ZERO )
		{
			result += POS_360;
		}
		if ( result > POS_180 )
		{
			result = POS_360 - result;
		}
		result /= POS_180;

		return result;
	}

	template<class _DataType>
	static _DataType trigAngleDiffRatio( const _DataType & from_angle, const _DataType & to_angle )
	{
		const static _DataType TWO( 2 );

		static _DataType from_angle_rad;
		static _DataType to_angle_rad;
		static _DataType d;

		from_angle_rad = degToRad( from_angle );
		to_angle_rad = degToRad( to_angle );
		d = sqrt( pow( cos( from_angle_rad ) - cos( to_angle_rad ), 2 ) + pow( sin( from_angle_rad ) - sin( to_angle_rad ), 2 ) );

		return d / TWO;
	}

	template<class _DataType>
	static _DataType probabilityDistribution( const _DataType & u, const _DataType & s, const _DataType & x )
	{
		return exp( -pow( x - u, 2 ) / ( 2 * pow( s, 2 ) ) ) / sqrt( 2 * M_PI * pow( s, 2 ) );
	}

	template<class _DataType>
	static void polarToEuler( _DataType &angle )
	{
		const static _DataType POS_360( 0 );
		const static _DataType NEG_90( -90 );
		const static _DataType ZERO( 0 );

		angle += NEG_90 + ( angle < ZERO ? POS_360 : ZERO );
	}

	template<class _DataType>
	static void eulerToPolar( _DataType &angle )
	{
		const static _DataType POS_360( 0 );
		const static _DataType POS_90( 90 );
		const static _DataType ZERO( 0 );

		angle += POS_90 + ( angle < ZERO ? POS_360 : ZERO );
	}

	template<typename _DataType>
	static _DataType angleDistRel( const _DataType & from_angle, const _DataType & to_angle ) //return the distance D from @angle1 to @a2 such that -180 < D < 180
	{
		const _DataType POS_360( 360 );
		const _DataType POS_180( 180 );
		const _DataType NEG_360( -360 );
		const _DataType NEG_180( -180 );
		const _DataType ZERO( 0 );

		static _DataType angle_diff;

		angle_diff = from_angle - to_angle;

		//this line does the following:
		//if c < -180, subtract 360 from a2; else if c > 180, add 360 to a2; else add 0 to a2
		//return a2 - angle1
		return ( to_angle + ( angle_diff < NEG_180 ? NEG_360 : ( angle_diff > POS_180 ? POS_360 : ZERO ) ) ) - from_angle;
	}

	template<typename _DataType>
	static void normalizeAngle( _DataType & angle ) //force @angle into 0 <= @angle <= 360
	{
		const static _DataType POS_360( 360 );

		angle = fmod( angle, POS_360 );
	}

	template<typename _DataType>
	static void capValue( _DataType & target, const _DataType & lower_cap, const _DataType & upper_cap ) //cap @target between @lower_cap and @upperCap
	{
		target = target < lower_cap ? lower_cap : ( target > upper_cap ? upper_cap : target ); //teh 1337
	}

	template<typename _DataType>
	static void capValue( _DataType & target, const _DataType & mag_cap ) //cap @target between -@magCap and @magCap
	{
		capValue( target, -mag_cap, mag_cap );
	}

	template<typename _DataType>
	static void capValueProp( _DataType & value1, _DataType & value2, const _DataType & mag_cap ) //scale @t1 and @t2 down by the same amount such that -@magCap <= t1, t2 <= magCap
	{
		const static bool dec_accuracy = HasDecAccuracy<_DataType>::val;

		static _DataType largest;
		static double scale;

		largest = fabs( value1 ) > fabs( value2 ) ? value1 : value2;

		if ( fabs( largest ) > fabs( mag_cap ) )
		{
			//_DataType diff = fabs(largest) - magCap;
			scale = fabs( (double) mag_cap ) / (double) largest;

			value1 = dec_accuracy ? value1 * scale : round( value1 * scale );
			value2 = dec_accuracy ? value2 * scale : round( value2 * scale );
		}
	}

	template<class __DataType>
	static typename std::enable_if<std::is_floating_point<__DataType>::value, __DataType>::type
	mod( const __DataType & numerator, const __DataType & denominator )
	{
		return fmod( numerator, denominator );
	}

	template<class __DataType>
	static typename std::enable_if<!std::is_floating_point<__DataType>::value, __DataType>::type
	mod( const __DataType & numerator, const __DataType & denominator )
	{
		return numerator % denominator;
	}

	static double gaussian( const double & x, const double & variance )
	{
		return ( 1 / sqrt( 2 * M_PI * variance ) ) * pow( M_E, -pow( x, 2 ) / ( 2 * variance ) );
	}

	static double normalizedGaussian( const double & x, const double & variance )
	{
		return pow( M_E, -pow( x, 2 ) / ( 2 * variance ) );
	}
}

#endif /* MATH_H_ */
