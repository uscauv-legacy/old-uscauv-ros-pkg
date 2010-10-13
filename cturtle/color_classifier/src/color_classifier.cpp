/*******************************************************************************
 *
 *      color_classifier
 * 
 *      Copyright (c) 2010, Edward T. Kaszubski
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

// for Mat
#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include "highgui.h"
// for BaseImageProc
#include <base_image_proc/base_image_proc.h>
// for cfg code
#include <color_classifier/ColorClassifierConfig.h>

typedef color_classifier::ColorClassifierConfig _ReconfigureType;
class ColorClassifier: public BaseImageProc<_ReconfigureType>
{

public:
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
private:

	ColorClassifier::ColorSpectrumHSV spectrum_hsv_;

	double ms_spatial_radius_;
	double ms_color_ratius_;
	int ms_max_level_;
	int ms_max_iter_;
	double ms_min_epsilon_;
	bool enable_thresholding_;
	bool enable_meanshift_;
	bool enable_color_filter_;

public:

	ColorClassifier( ros::NodeHandle & nh ) :
		BaseImageProc<_ReconfigureType> ( nh )
	{
		//
	}

	~ColorClassifier()
	{
		//
	}

	double wrapValue( double value, double min, double max )
	{
		while ( value < min )
		{
			value += ( max - min );
		}
		while ( value > max )
		{
			value -= ( max - min );
		}
		return value;
	}


	//using BaseImageProc<_ReconfigureType>::reconfigureCB;
	void reconfigureCB( _ReconfigureType &config, uint32_t level )
	{
		spectrum_hsv_.red.min = wrapValue( config.red_h_c - config.red_h_r, 0.0, 360.0 );
		spectrum_hsv_.red.max = wrapValue( config.red_h_c + config.red_h_r, 0.0, 360.0 );

		spectrum_hsv_.orange.min = wrapValue( config.orange_h_c - config.orange_h_r, 0.0, 360.0 );
		spectrum_hsv_.orange.max = wrapValue( config.orange_h_c + config.orange_h_r, 0.0, 360.0 );

		spectrum_hsv_.yellow.min = wrapValue( config.yellow_h_c - config.yellow_h_r, 0.0, 360.0 );
		spectrum_hsv_.yellow.max = wrapValue( config.yellow_h_c + config.yellow_h_r, 0.0, 360.0 );

		spectrum_hsv_.green.min = wrapValue( config.green_h_c - config.green_h_r, 0.0, 360.0 );
		spectrum_hsv_.green.max = wrapValue( config.green_h_c + config.green_h_r, 0.0, 360.0 );

		spectrum_hsv_.blue.min = wrapValue( config.blue_h_c - config.blue_h_r, 0.0, 360.0 );
		spectrum_hsv_.blue.max = wrapValue( config.blue_h_c + config.blue_h_r, 0.0, 360.0 );

		ms_spatial_radius_ = config.ms_spatial_radius;
		ms_color_ratius_ = config.ms_color_ratius;
		ms_max_level_ = config.ms_max_level;
		ms_max_iter_ = config.ms_max_iter;
		ms_min_epsilon_ = config.ms_min_epsilon;

		enable_color_filter_ = config.enable_color_filter;
		enable_meanshift_ = config.enable_meanshift;
		enable_thresholding_ = config.enable_thresholding;


		/*spectrum_hsv_.red.min = config.red_h_min;
		 spectrum_hsv_.red.max = config.red_h_max;

		 spectrum_hsv_.orange.min = config.orange_h_min;
		 spectrum_hsv_.orange.max = config.orange_h_max;

		 spectrum_hsv_.yellow.min = config.yellow_h_min;
		 spectrum_hsv_.yellow.max = config.yellow_h_max;

		 spectrum_hsv_.green.min = config.green_h_min;
		 spectrum_hsv_.green.max = config.green_h_max;

		 spectrum_hsv_.blue.min = config.blue_h_min;
		 spectrum_hsv_.blue.max = config.blue_h_max;*/

		spectrum_hsv_.black_l_threshold = config.black_l_threshold;
		spectrum_hsv_.white_l_threshold = config.white_l_threshold;
		spectrum_hsv_.unknown_s_threshold = config.unknown_s_threshold;

		reconfigure_initialized_ = true;
	}

	virtual cv::Mat processImage( IplImage * ipl_img )
	{
		IplImage * mean_shifted_img = ipl_img;
		if ( enable_meanshift_ )
		{
			cvPyrMeanShiftFiltering( ipl_img, mean_shifted_img, ms_spatial_radius_, ms_color_ratius_, ms_max_level_, cvTermCriteria( CV_TERMCRIT_ITER + CV_TERMCRIT_EPS, ms_max_iter_, ms_min_epsilon_ ) );
		}
		cv_img_ = cv::Mat( mean_shifted_img );
		cv::Mat hsv_img;

		if ( enable_color_filter_ )
		{
			for ( int y = 0; y < cv_img_.size().height; y++ )
			{
				for ( int x = 0; x < cv_img_.size().width; x++ )
				{
					// get pixel data
					cv::Vec3b pixel = cv_img_.at<cv::Vec3b> ( cv::Point( x, y ) );
					// set pixel data
					cv_img_.at<cv::Vec3b> ( cv::Point( x, y ) ) = filterColor( pixel );
				}
			}
		}

		if ( enable_thresholding_ )
		{
			cv::cvtColor( cv_img_, hsv_img, CV_BGR2HLS);
			for ( int y = 0; y < hsv_img.size().height; y++ )
			{
				for ( int x = 0; x < hsv_img.size().width; x++ )
				{
					// get pixel data
					cv::Vec3b pixel = hsv_img.at<cv::Vec3b> ( cv::Point( x, y ) );
					// set pixel data
					cv_img_.at<cv::Vec3b> ( cv::Point( x, y ) ) = classifyPixel( pixel );
				}
			}
		}

		return cv_img_;
	}

	inline cv::Vec3b filterColor( cv::Vec3b & pixel )
	{
		cv::Vec3b result;
		//result[2] = (int) round( (double) pixel[2] * 0.937254902 );
		//result[1] = (int) round( (double) pixel[1] * 0.635294118 );
		//result[0] = (int) round( (double) pixel[0] * 0.521568627 );
		result[2] = std::max( pixel[2] - 16, 0 );
		result[1] = std::max( pixel[1] - 93, 0 );
		result[0] = std::max( pixel[0] - 122, 0 );
		return result;
	}

	inline cv::Vec3b classifyPixel( cv::Vec3b & pixel )
	{
		double h = 360.0 * (double) pixel[0] / 255.0;
		double l = 100.0 * (double) pixel[1] / 255.0;
		double s = 100.0 * (double) pixel[2] / 255.0;

		if ( s < spectrum_hsv_.unknown_s_threshold ) return OutputColorRGB::unknown;

		if ( l < spectrum_hsv_.black_l_threshold ) return OutputColorRGB::black;
		if ( l > spectrum_hsv_.white_l_threshold ) return OutputColorRGB::white;

		// any color with a value less than this is considered black
		//if ( v < spectrum_hsv_.black_v_threshold ) return OutputColorRGB::black;

		// any color with a combined value-saturation ( v * ( 1-s ) ) greater than this is considered white
		//if ( v * ( 1.0 - s ) > spectrum_hsv_.white_combined_sv_threshold ) return OutputColorRGB::white;

		// any color with a combined value-saturation ( v * s ) less than this is considered black
		//if ( s * v < spectrum_hsv_.black_combined_sv_threshold ) return OutputColorRGB::black;

		if ( spectrum_hsv_.red.isInRange( h, true ) ) return OutputColorRGB::red;
		if ( spectrum_hsv_.orange.isInRange( h, true ) ) return OutputColorRGB::orange;
		if ( spectrum_hsv_.yellow.isInRange( h, true ) ) return OutputColorRGB::yellow;
		if ( spectrum_hsv_.green.isInRange( h, true ) ) return OutputColorRGB::green;
		if ( spectrum_hsv_.blue.isInRange( h, true ) ) return OutputColorRGB::blue;
		return OutputColorRGB::unknown;
	}
};

// bgr
const cv::Vec3b ColorClassifier::OutputColorRGB::red = cv::Vec3b( 0, 0, 255 );
const cv::Vec3b ColorClassifier::OutputColorRGB::orange = cv::Vec3b( 0, 128, 255 );
const cv::Vec3b ColorClassifier::OutputColorRGB::yellow = cv::Vec3b( 0, 255, 255 );
const cv::Vec3b ColorClassifier::OutputColorRGB::green = cv::Vec3b( 0, 255, 0 );
const cv::Vec3b ColorClassifier::OutputColorRGB::blue = cv::Vec3b( 255, 0, 0 );
const cv::Vec3b ColorClassifier::OutputColorRGB::black = cv::Vec3b( 0, 0, 0 );
const cv::Vec3b ColorClassifier::OutputColorRGB::white = cv::Vec3b( 255, 255, 255 );
const cv::Vec3b ColorClassifier::OutputColorRGB::unknown = cv::Vec3b( 128, 128, 128 );

int main( int argc, char ** argv )
{
	ros::init( argc, argv, "color_classifier" );
	ros::NodeHandle nh;

	ColorClassifier color_classifier( nh );
	color_classifier.spin();

	return 0;
}
