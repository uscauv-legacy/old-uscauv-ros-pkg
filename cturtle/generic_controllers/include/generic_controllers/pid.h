/*******************************************************************************
 *
 *      pid
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

#ifndef PID_H_
#define PID_H_

#include <time.h>
#include <vector>
#include <math.h>

#include <common_utils/time.h>

template<class __StorageDataType = double, class __TimeDataType = double>
class IPidBase
{
public:
	typedef __StorageDataType _StorageDataType;
	typedef __TimeDataType _TimeDataType;

	struct Settings
	{
		const static _StorageDataType DEF_p_ = 1;
		const static _StorageDataType DEF_i_ = 0;
		const static _StorageDataType DEF_d_ = 0.05;
		const static _StorageDataType DEF_i_min_ = -1;
		const static _StorageDataType DEF_i_max_ = 1;
		const static _StorageDataType DEF_e_min_ = 0;
		const static _StorageDataType DEF_e_max_ = 0;

		Settings( _StorageDataType p_ = DEF_p_, _StorageDataType i_ = DEF_i_, _StorageDataType d_ = DEF_d_, _StorageDataType i_min_ = DEF_i_min_, _StorageDataType i_max_ = DEF_i_max_,
				_StorageDataType e_min_ = DEF_e_min_, _StorageDataType e_max_ = DEF_e_max_ ) :
			p( p_ ), i( i_ ), d( d_ ), i_min( i_min_ ), i_max( i_max_ ), e_min( e_min_ ), e_max( e_max_ )
		{
			//
		}

		void apply()
		{

		}

		_StorageDataType p, i, d, i_min, i_max, e_min, e_max;
	};

	virtual void init() = 0;

	virtual void reset() = 0;

	// update using a given dt
	virtual _StorageDataType update( _StorageDataType error, _TimeDataType dt, bool update_time = true ) = 0;

	// calculate dt automatically
	virtual _StorageDataType update( _StorageDataType error ) = 0;

	virtual _TimeDataType updateTime() = 0;
};

template<class __StorageDataType = double, class __TimeDataType = double>
class PidBase : public IPidBase<__StorageDataType, __TimeDataType> {
public:
typedef __StorageDataType _StorageDataType;
typedef __TimeDataType _TimeDataType;
typedef IPidBase<__StorageDataType, __TimeDataType> _IPidBase;
typedef typename _IPidBase::Settings _Settings;

_Settings settings;

_StorageDataType output, integral, last_error;

bool time_initialized, error_initialized;
_TimeDataType current_time, last_update_time;

PidBase( _StorageDataType output_ = 0 ) :
output( output_ )
{
	//
	}

	virtual void init()
	{
		update( 0 );
	}

	virtual void reset()
	{
		output = 0;
		integral = 0;
		time_initialized = false;
		error_initialized = false;
	}

	// update using a given dt
	virtual _StorageDataType update( _StorageDataType error, _TimeDataType dt, bool update_time = true )
	{
		printf( "update( %f %f )\n", error, dt );

		// to prevent insane P values, limit the error to within the specified bounds
		if ( settings.e_min != 0 && error < settings.e_min ) error = settings.e_min;
		if ( settings.e_max != 0 && error > settings.e_max ) error = settings.e_max;

		//printf( "capped error: %f\n", error );

		//printf( "settings: %f %f\n", settings.e_min, settings.e_max );

		// register the loop was updated
		if ( update_time ) updateTime();

		if ( !error_initialized )
		{
			error_initialized = true;
			last_error = error;
			printf( "error initialized: %f\n", last_error );
		}

		output = settings.p * error;
		if ( dt > 0 )
		{
			if ( fabs( settings.i ) > 0 )
			{
				integral += error * dt;

				// make sure the integral term doesn't go outside of [i_min, i_max]
				integral = integral > settings.i_max ? settings.i_max : integral < settings.i_min ? settings.i_min : integral;

				output += settings.i * integral;
			}

			printf( "last_error %f error %f dE %f dT %f\n", last_error, error, last_error - error, dt );

			output += settings.d > 0 ? settings.d * ( error - last_error ) / dt : 0;
		}
		last_error = error;

		printf( "error: %f integral: %f output: %f\n", error, integral, output );

		return output;
	}

	// calculate dt automatically
	virtual _StorageDataType update( _StorageDataType error )
	{
		printf( "update( %f )\n", error );

		if ( !time_initialized ) updateTime();
		else update( error, updateTime(), false );

		return output;
	}

	virtual _TimeDataType updateTime()
	{
		last_update_time = time_initialized ? current_time : _TimeDataType( time_utils::getTimeInSecs() );
		current_time = _TimeDataType( time_utils::getTimeInSecs() );
		time_initialized = true;
		return current_time - last_update_time;
	}

	void applySettings()
	{
		//
	}

	void applySettings( _Settings settings_ )
	{
		settings = settings_;
		applySettings();
	}
};

template<class __PidBaseType, unsigned int __Dim__ = 1>
class IPid
{
public:
	typedef typename __PidBaseType::_StorageDataType _StorageDataType;
	typedef typename __PidBaseType::_TimeDataType _TimeDataType;

	typedef __PidBaseType _PidBaseType;
	typedef typename _PidBaseType::Settings _Settings;

	virtual void applySettings( std::vector<_Settings> settings_array ) = 0;

	virtual void applySettings( unsigned int i, _Settings settings ) = 0;

	virtual void reset() = 0;

	virtual std::vector<_StorageDataType> update( std::vector<_StorageDataType> error, _TimeDataType dt = -1 ) = 0;

	virtual const _PidBaseType & operator[]( unsigned int index ) const = 0;

	virtual _PidBaseType & operator[]( unsigned int index ) = 0;
};

template<class __PidBaseType>
class IPid<__PidBaseType, 1>
{
public:
	typedef typename __PidBaseType::_StorageDataType _StorageDataType;
	typedef typename __PidBaseType::_TimeDataType _TimeDataType;

	typedef __PidBaseType _PidBaseType;
	typedef typename _PidBaseType::Settings _Settings;

	virtual void applySettings( _Settings settings ) = 0;

	virtual void reset() = 0;

	virtual _StorageDataType update( _StorageDataType error, _TimeDataType dt = -1 ) = 0;
};

template<class __PidBaseType, unsigned int __Dim__ = 1>
class Pid : public IPid<__PidBaseType, __Dim__> {
public:
typedef typename __PidBaseType::_StorageDataType _StorageDataType;
typedef typename __PidBaseType::_TimeDataType _TimeDataType;

typedef __PidBaseType _PidBaseType;
typedef typename _PidBaseType::Settings _Settings;

protected:
_PidBaseType pids[__Dim__];
std::vector<_StorageDataType> outputs;

typedef std::vector<_Settings> _SettingsArray;

public:
Pid()
{
	//applySettings( settings_array );
	}
	virtual ~Pid()
	{
		//
	}

	void applySettings( _SettingsArray settings_array )
	{
		for ( unsigned int i = 0; i < __Dim__; ++i )
		{
			pids[i].applySettings( settings_array[i] );
		}
	}

	void applySettings( unsigned int i, _Settings settings )
	{
		pids[i].applySettings( settings );
	}

	virtual void reset()
	{
		for ( unsigned int i = 0; i < __Dim__; ++i )
		{
			pids[i].reset();
		}
	}

	virtual std::vector<_StorageDataType> update( std::vector<_StorageDataType> error, _TimeDataType dt = -1 )
	{
		for ( unsigned int i = 0; i < __Dim__; ++i )
		{
			outputs[i] = dt > 0 ? pids[i].update( error[i], dt ) : pids[i].update( error[i] );
		}

		return outputs;
	}

	const _PidBaseType & operator[]( unsigned int index ) const
	{
		return pids[index];
	}
	_PidBaseType & operator[]( unsigned int index )
	{
		return pids[index];
	}

};

// if we want a single-component PID, just inherit from PidBase
template<class __PidBaseType>
class Pid<__PidBaseType, 1> : public IPid<__PidBaseType, 1>, public __PidBaseType
{
public:
	typedef typename __PidBaseType::_StorageDataType _StorageDataType;
	typedef typename __PidBaseType::_TimeDataType _TimeDataType;

	typedef __PidBaseType _PidBaseType;
	typedef typename _PidBaseType::Settings _Settings;

public:
	Pid() : __PidBaseType()
	{

	}
	virtual ~Pid()
	{
		//
	}
};

template<class __PidBaseType>
class Pid3D : public Pid<__PidBaseType, 3>
{
public:
	typedef typename __PidBaseType::_StorageDataType _StorageDataType;
	typedef typename __PidBaseType::_TimeDataType _TimeDataType;

	typedef __PidBaseType _PidBaseType;
	typedef typename _PidBaseType::Settings _Settings;
	typedef std::vector<_Settings> _SettingsArray;

	typedef Pid<_PidBaseType, 1> _Pid;

	// allocate storage for a 3D pid
		typedef Pid<_PidBaseType, 3> _Pid3D;

		_PidBaseType * x, *y, *z;

		Pid3D() :
		_Pid3D(), x( &this->pids[0] ), y( &this->pids[1] ), z( &this->pids[2] )
		{
			//
		}

		virtual ~Pid3D()
		{
			//
		}
	};

template<class __PidBaseType>
class Pid6D : public Pid<__PidBaseType, 6>
{
public:
	typedef typename __PidBaseType::_StorageDataType _StorageDataType;
	typedef typename __PidBaseType::_TimeDataType _TimeDataType;

	typedef __PidBaseType _PidBaseType;
	typedef typename _PidBaseType::Settings _Settings;
	typedef std::vector<_Settings> _SettingsArray;

	typedef Pid<_PidBaseType, 1> _Pid;
	typedef Pid3D<_PidBaseType> _Pid3D;

	// allocate storage for a 6D pid
		typedef Pid<_PidBaseType, 6> _Pid6D;

		_PidBaseType * linear_x, * linear_y, * linear_z, * angular_x, * angular_y, * angular_z;

		Pid6D() : _Pid6D(), linear_x( &this->pids[0] ), linear_y( &this->pids[1] ), linear_z( &this->pids[2] ), angular_x( &this->pids[3] ), angular_y( &this->pids[4] ),angular_z( &this->pids[5] )
		{
			//
		}

		virtual ~Pid6D()
		{
			//
		}
	};

#endif /* PID_H_ */
