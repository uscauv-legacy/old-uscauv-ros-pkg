/*******************************************************************************
 *
 *      feature
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
 *      * Neither the name of "common_utils-RelWithDebInfo@common_utils" nor the names of its
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

#ifndef FEATURE_H_
#define FEATURE_H_

#include <common_utils/vec.h>
#include <common_utils/math.h>

typedef unsigned int _DimType;

struct DistanceType
{
	const static _DimType EUCLIDIAN = 0;
	const static _DimType GAUSSIAN = 1;
};

// default template for FeatureBase with decimal accuracy
template<class __DataType>
class FeatureBase
{
public:

	typedef __DataType _DataType;
	typedef FeatureBase<__DataType> _FeatureBase;

	__DataType data_;

	FeatureBase( const __DataType & data = __DataType( 0 ) ) :
	data_( data )
	{
		//
	}

	__DataType distanceTo( const _FeatureBase & other, const __DataType & radius = __DataType( 0 ) ) const
	{
		// 0 - 179 = -179
		__DataType dist = other.data_ - data_;
		if ( radius > 0 && fabs( dist ) > radius )
	    {
	    	dist = math_utils::mod( __DataType( fabs( dist ) ),
	    	                        radius ) - radius;
	    }
	    return dist;
	}

	static __DataType caltulateEuclidianDistanceComponent( const __DataType & dist )
	{
		return dist;
	}

	template<class _WeightType>
	static __DataType calculateGaussianDistanceComponent( const __DataType & dist, const _WeightType & variance )
	{
		if( variance == 0 ) return dist == 0 ? 1.0 : 0.0;
		return math_utils::gaussian( dist, variance );
	}

	template<class _WeightType>
	static void calculateDistanceComponent( __DataType & total_distance, const _FeatureBase & current, const _FeatureBase & other, const __DataType & radius, const _WeightType & weight, const _WeightType & variance, _DimType distance_type, bool initialize_total_distance )
	{
		__DataType dist = current.distanceTo( other, radius );

		switch( distance_type )
		{
		case DistanceType::EUCLIDIAN:
			if( initialize_total_distance ) total_distance = 0.0;
			total_distance += pow( caltulateEuclidianDistanceComponent( dist ) * weight, 2 );
			break;
		case DistanceType::GAUSSIAN:
			if( initialize_total_distance ) total_distance = 1.0;
			total_distance *= calculateGaussianDistanceComponent( dist, variance );
			break;
		}
	}
};

template<class __DataType, _DimType __Dim__ = 1>
class Feature: public Vec<FeatureBase<__DataType >, __Dim__>
{
public:
	typedef Vec<FeatureBase<__DataType >, __Dim__> _Vec;
	typedef std::array<__DataType, __Dim__> _ArrayType;
	typedef __DataType _DataType;
	typedef Feature<_DataType, __Dim__> _Feature;
	typedef FeatureBase<_DataType> _FeatureBase;

	Feature( const _ArrayType data ) :
	_Vec()
	{
		for ( _DimType i = 0; i < __Dim__; ++i )
		{
			this->data_[i] = _FeatureBase( data[i] );
		}
	}

	Feature() :
	_Vec()
	{

	}

	template<class _WeightType>
	__DataType distance( const _Feature & other, _ArrayType & radii, std::array<_WeightType, __Dim__> & weights, std::array<_WeightType, __Dim__> variances = std::array<_WeightType, __Dim__>(), _DimType distance_type = DistanceType::GAUSSIAN )
	{
		__DataType total_distance = 0;

		/*_WeightType total_weight = 0.0;
		for ( _DimType i = 0; i < __Dim__; ++i )
		{
			total_weight += weights[i];
		}

		const _WeightType weight_norm_scale = 1.0 / total_weight;*/

		for ( _DimType i = 0; i < __Dim__; ++i )
		{
			if( weights[i] > 0 ) _FeatureBase::calculateDistanceComponent( total_distance, this->data_[i], other.data_[i], radii[i], weights[i], variances[i], distance_type, i == 0 );
		}

		if( distance_type == DistanceType::EUCLIDIAN ) return sqrt( total_distance );

		return sqrt( total_distance );
	}

	template<class _WeightType>
	__DataType distance( const _Feature & other, _ArrayType & radii, std::array<_WeightType, __Dim__> & weights )
	{
		__DataType total_distance = 0;

		for ( _DimType i = 0; i < __Dim__; ++i )
		{
			if( weights[i] > 0 ) _FeatureBase::calculateDistanceComponent( total_distance, this->data_[i], other.data_[i], radii[i], weights[i], 0.0, DistanceType::EUCLIDIAN, i == 0 );
		}

		return sqrt( total_distance );
	}

	template<class _WeightType>
	__DataType distance( const _Feature & other, _ArrayType & radii = _ArrayType(), const _WeightType & uniform_weight = 1.0 )
	{
		__DataType total_distance = 0;

		if ( uniform_weight > 0 )
		{
			for ( _DimType i = 0; i < __Dim__; ++i )
			{
				_FeatureBase::calculateDistanceComponent( total_distance, this->data_[i], other.data_[i], radii[i], uniform_weight, 0.0, DistanceType::EUCLIDIAN, i == 0 );
			}
		}

		return sqrt( total_distance );
	}

	__DataType distance( const _Feature & other )
	{
		__DataType total_distance = 0;

		for ( _DimType i = 0; i < __Dim__; ++i )
		{
			_FeatureBase::calculateDistanceComponent( total_distance, this->data_[i], other.data_[i], 0.0, 1.0, 0.0, DistanceType::EUCLIDIAN, i == 0 );
		}

		return total_distance;
	}

	/*template<class _WeightType>
	__DataType distance( const _Feature & other, const __DataType & uniform_radius = __DataType( 0 ), const _WeightType & uniform_weight = _WeightType( 1 ), _DimType distance_type = DistanceType::EUCLIDIAN )
	{
		__DataType total_distance;

		for ( _DimType i = 0; i < __Dim__; ++i )
		{
			_FeatureBase::calculateDistanceComponent( total_distance, this->data_[i], other.data_[i], uniform_radius, uniform_weight, distance_type, i == 0 );
		}

		return sqrt( total_distance );
	}

	template<class _WeightType>
	__DataType distance( const _Feature & other, const __DataType & uniform_radius, std::array<_WeightType, __Dim__> weights, _DimType distance_type = DistanceType::EUCLIDIAN )
	{
		__DataType total_distance;

		for ( _DimType i = 0; i < __Dim__; ++i )
		{
			_FeatureBase::calculateDistanceComponent( total_distance, this->data_[i], other.data_[i], uniform_radius, weights[i], distance_type, i == 0 );
		}

		return sqrt( total_distance );
	}*/
};

template<class __DataType>
class Feature<__DataType, 1>
{
public:
	typedef __DataType _DataType;
	typedef FeatureBase<_DataType> _FeatureBase;
	typedef Feature<__DataType, 1> _Feature;

	const static _DimType _Dim_ = 1;
	_FeatureBase data_;

	Feature( const __DataType & data = __DataType( 0 ) ) :
			data_( data )
	{
		//
	}

	__DataType distance( const _Feature & other,
	                     __DataType radius = __DataType( 0 ) )
	{
		return data_.distanceTo( other.data_, radius );
	}
};

#endif /* FEATURE_H_ */
