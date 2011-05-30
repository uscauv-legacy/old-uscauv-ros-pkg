/*******************************************************************************
 *
 *      control_common
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

#ifndef CONTROL_COMMON_H_
#define CONTROL_COMMON_H_

#include <generic_controllers/pid.h> // for Pid
// for dynamic reconfigure server
#include <dynamic_reconfigure/server.h>
#include <seabee3_common/../../cfg/cpp/seabee3_common/PidConfig.h>

template<class __StorageDataType = double, class __TimeDataType = double>
class ConfiguredPidBase : public IPidBase<__StorageDataType, __TimeDataType>
{
public:
	typedef __StorageDataType _StorageDataType;
	typedef __TimeDataType _TimeDataType;
	typedef seabee3_common::PidConfig _ReconfigureType;

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

		_StorageDataType p, i, d, i_min, i_max, e_min, e_max;
	};

	Settings settings;
	_StorageDataType output, integral, last_error;

protected:
	ros::NodeHandle nh_priv;
	dynamic_reconfigure::Server<_ReconfigureType> reconfigure_srv;
	typename dynamic_reconfigure::Server<_ReconfigureType>::CallbackType reconfigure_callback;

	bool time_initialized, error_initialized;
	_TimeDataType current_time, last_update_time;

	// workaround for reconfigure ussues
	bool reconfigure_initialized;
	_ReconfigureType initial_config_params;
	uint32_t initial_config_level;

public:
	ConfiguredPidBase( _StorageDataType p_ = Settings::DEF_p_, _StorageDataType i_ = Settings::DEF_i_, _StorageDataType d_ = Settings::DEF_d_,
			_StorageDataType i_min_ = Settings::DEF_i_min_, _StorageDataType i_max_ = Settings::DEF_i_max_, _StorageDataType e_min_ = Settings::DEF_e_min_,
			_StorageDataType e_max_ = Settings::DEF_e_max_, _StorageDataType output_ = 0 ) :
		nh_priv( "~" ), settings( p_, i_, d_, i_min_, i_max_, e_min_, e_max_ ), output( output_ ), integral( 0 ), last_error( 0 ), time_initialized( false ), error_initialized( false ),
				current_time( 0 ), last_update_time( 0 )
	{
		//
	}

	ConfiguredPidBase( Settings settings_, _StorageDataType output_ = 0 ) :
		settings( settings_ ), output( output_ )
	{
		//
	}

	ConfiguredPidBase( ros::NodeHandle & nh_ ) :
		nh_priv( "~" )
	{
		reconfigure_callback = boost::bind( &ConfiguredPidBase::reconfigureCB, this, _1, _2 );
		reconfigure_srv.setCallback( reconfigure_callback );
	}

	void reconfigureCB( _ReconfigureType &config, uint32_t level )
	{
		initial_config_params = config;
		initial_config_level = level;

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
};

#endif /* CONTROL_COMMON_H_ */
