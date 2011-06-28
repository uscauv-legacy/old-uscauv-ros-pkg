/*******************************************************************************
 *
 *      color_classifier
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
 *      * Neither the name of "color_classifier-RelWithDebInfo@color_classifier" nor the names of its
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

#ifndef COLOR_CLASSIFIER_H_
#define COLOR_CLASSIFIER_H_



/* others */
// for Mat
#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include "highgui.h"
// for BaseImageProc
#include <base_image_proc/base_image_proc.h>
// for cfg code
#include <color_classifier/ColorClassifierConfig.h>
// for colors
#include <color_defs/colors.h>

typedef color_classifier::ColorClassifierConfig _ReconfigureType;
typedef double _DataType;

class ColorClassifier: public BaseImageProc<_ReconfigureType>
{

public:
	typedef std::array<ThresholdedColor, OutputColorRGB::NUM_COLORS> _ThresholdedColorArrayType;
	typedef std::array<IplImage *, OutputColorRGB::NUM_COLORS> _IplImagePtrArrayType;
	typedef std::array<sensor_msgs::CvBridge, OutputColorRGB::NUM_COLORS> _CvBridgeArrayType;
	typedef std::array<image_transport::Publisher, OutputColorRGB::NUM_COLORS> _ImageTransportPublisherArrayType;

	/* reconfigure params */
	_ThresholdedColorArrayType target_colors_;
	_IplImagePtrArrayType distance_images_;
	_CvBridgeArrayType image_bridges_;
	_ImageTransportPublisherArrayType image_transport_publishers_;

	double ms_spatial_radius_;
	double ms_color_radius_;int ms_max_level_;int ms_max_iter_;
	double ms_min_epsilon_;

	bool enable_meanshift_;

	bool enable_thresholding_;
	double threshold_value_;
	double threshold_max_value_;
	unsigned char threshold_type_;

	/* constructor params */
	bool process_image_initialized_;

public:

	ColorClassifier( ros::NodeHandle & nh ) :
			BaseImageProc<_ReconfigureType>( nh ), process_image_initialized_( false )
	{
		// create publishers
		for ( unsigned int i = 0; i < image_transport_publishers_.size(); ++i )
		{
			image_transport_publishers_[i] = this->image_transport_.advertise( "components/" + OutputColorRGB::getColorName( i ),
			                                                                   1 );
		}

		initCfgParams();
	}

	virtual ~ColorClassifier()
	{
		//
	}

	// virtual
	void reconfigureCB( _ReconfigureType &config,
	                    uint32_t level )
	{
		target_colors_[OutputColorRGB::Id::red].enabled = config.red_filter_enabled;
		*target_colors_[OutputColorRGB::Id::red].color.i_ = config.red_hue;
		*target_colors_[OutputColorRGB::Id::red].color.j_ = config.red_sat;
		*target_colors_[OutputColorRGB::Id::red].color.k_ = config.red_val;
		target_colors_[OutputColorRGB::Id::red].threshold = config.red_threshold;
		*target_colors_[OutputColorRGB::Id::red].weight.i_ = config.red_hue_weight;
		*target_colors_[OutputColorRGB::Id::red].weight.j_ = config.red_sat_weight;
		*target_colors_[OutputColorRGB::Id::red].weight.k_ = config.red_val_weight;
		*target_colors_[OutputColorRGB::Id::red].variance.i_ = config.red_hue_variance;
		*target_colors_[OutputColorRGB::Id::red].variance.j_ = config.red_sat_variance;
		*target_colors_[OutputColorRGB::Id::red].variance.k_ = config.red_val_variance;

		target_colors_[OutputColorRGB::Id::orange].enabled = config.orange_filter_enabled;
		*target_colors_[OutputColorRGB::Id::orange].color.i_ = config.orange_hue;
		*target_colors_[OutputColorRGB::Id::orange].color.j_ = config.orange_sat;
		*target_colors_[OutputColorRGB::Id::orange].color.k_ = config.orange_val;
		target_colors_[OutputColorRGB::Id::orange].threshold = config.orange_threshold;
		*target_colors_[OutputColorRGB::Id::orange].weight.i_ = config.orange_hue_weight;
		*target_colors_[OutputColorRGB::Id::orange].weight.j_ = config.orange_sat_weight;
		*target_colors_[OutputColorRGB::Id::orange].weight.k_ = config.orange_val_weight;
		*target_colors_[OutputColorRGB::Id::orange].variance.i_ = config.orange_hue_variance;
		*target_colors_[OutputColorRGB::Id::orange].variance.j_ = config.orange_sat_variance;
		*target_colors_[OutputColorRGB::Id::orange].variance.k_ = config.orange_val_variance;

		target_colors_[OutputColorRGB::Id::yellow].enabled = config.yellow_filter_enabled;
		*target_colors_[OutputColorRGB::Id::yellow].color.i_ = config.yellow_hue;
		*target_colors_[OutputColorRGB::Id::yellow].color.j_ = config.yellow_sat;
		*target_colors_[OutputColorRGB::Id::yellow].color.k_ = config.yellow_val;
		target_colors_[OutputColorRGB::Id::yellow].threshold = config.yellow_threshold;
		*target_colors_[OutputColorRGB::Id::yellow].weight.i_ = config.yellow_hue_weight;
		*target_colors_[OutputColorRGB::Id::yellow].weight.j_ = config.yellow_sat_weight;
		*target_colors_[OutputColorRGB::Id::yellow].weight.k_ = config.yellow_val_weight;
		*target_colors_[OutputColorRGB::Id::yellow].variance.i_ = config.yellow_hue_variance;
		*target_colors_[OutputColorRGB::Id::yellow].variance.j_ = config.yellow_sat_variance;
		*target_colors_[OutputColorRGB::Id::yellow].variance.k_ = config.yellow_val_variance;

		target_colors_[OutputColorRGB::Id::green].enabled = config.green_filter_enabled;
		*target_colors_[OutputColorRGB::Id::green].color.i_ = config.green_hue;
		*target_colors_[OutputColorRGB::Id::green].color.j_ = config.green_sat;
		*target_colors_[OutputColorRGB::Id::green].color.k_ = config.green_val;
		target_colors_[OutputColorRGB::Id::green].threshold = config.green_threshold;
		*target_colors_[OutputColorRGB::Id::green].weight.i_ = config.green_hue_weight;
		*target_colors_[OutputColorRGB::Id::green].weight.j_ = config.green_sat_weight;
		*target_colors_[OutputColorRGB::Id::green].weight.k_ = config.green_val_weight;
		*target_colors_[OutputColorRGB::Id::green].variance.i_ = config.green_hue_variance;
		*target_colors_[OutputColorRGB::Id::green].variance.j_ = config.green_sat_variance;
		*target_colors_[OutputColorRGB::Id::green].variance.k_ = config.green_val_variance;

		target_colors_[OutputColorRGB::Id::blue].enabled = config.blue_filter_enabled;
		*target_colors_[OutputColorRGB::Id::blue].color.i_ = config.blue_hue;
		*target_colors_[OutputColorRGB::Id::blue].color.j_ = config.blue_sat;
		*target_colors_[OutputColorRGB::Id::blue].color.k_ = config.blue_val;
		target_colors_[OutputColorRGB::Id::blue].threshold = config.blue_threshold;
		*target_colors_[OutputColorRGB::Id::blue].weight.i_ = config.blue_hue_weight;
		*target_colors_[OutputColorRGB::Id::blue].weight.j_ = config.blue_sat_weight;
		*target_colors_[OutputColorRGB::Id::blue].weight.k_ = config.blue_val_weight;
		*target_colors_[OutputColorRGB::Id::blue].variance.i_ = config.blue_hue_variance;
		*target_colors_[OutputColorRGB::Id::blue].variance.j_ = config.blue_sat_variance;
		*target_colors_[OutputColorRGB::Id::blue].variance.k_ = config.blue_val_variance;

		target_colors_[OutputColorRGB::Id::black].enabled = config.black_filter_enabled;
		*target_colors_[OutputColorRGB::Id::black].color.i_ = config.black_hue;
		*target_colors_[OutputColorRGB::Id::black].color.j_ = config.black_sat;
		*target_colors_[OutputColorRGB::Id::black].color.k_ = config.black_val;
		target_colors_[OutputColorRGB::Id::black].threshold = config.black_threshold;
		*target_colors_[OutputColorRGB::Id::black].weight.i_ = config.black_hue_weight;
		*target_colors_[OutputColorRGB::Id::black].weight.j_ = config.black_sat_weight;
		*target_colors_[OutputColorRGB::Id::black].weight.k_ = config.black_val_weight;
		*target_colors_[OutputColorRGB::Id::black].variance.i_ = config.black_hue_variance;
		*target_colors_[OutputColorRGB::Id::black].variance.j_ = config.black_sat_variance;
		*target_colors_[OutputColorRGB::Id::black].variance.k_ = config.black_val_variance;

		target_colors_[OutputColorRGB::Id::white].enabled = config.white_filter_enabled;
		*target_colors_[OutputColorRGB::Id::white].color.i_ = config.white_hue;
		*target_colors_[OutputColorRGB::Id::white].color.j_ = config.white_sat;
		*target_colors_[OutputColorRGB::Id::white].color.k_ = config.white_val;
		target_colors_[OutputColorRGB::Id::white].threshold = config.white_threshold;
		*target_colors_[OutputColorRGB::Id::white].weight.i_ = config.white_hue_weight;
		*target_colors_[OutputColorRGB::Id::white].weight.j_ = config.white_sat_weight;
		*target_colors_[OutputColorRGB::Id::white].weight.k_ = config.white_val_weight;
		*target_colors_[OutputColorRGB::Id::white].variance.i_ = config.white_hue_variance;
		*target_colors_[OutputColorRGB::Id::white].variance.j_ = config.white_sat_variance;
		*target_colors_[OutputColorRGB::Id::white].variance.k_ = config.white_val_variance;

		ms_spatial_radius_ = config.ms_spatial_radius;
		ms_color_radius_ = config.ms_color_ratius;
		ms_max_level_ = config.ms_max_level;
		ms_max_iter_ = config.ms_max_iter;
		ms_min_epsilon_ = config.ms_min_epsilon;

		enable_meanshift_ = config.enable_meanshift;

		enable_thresholding_ = config.enable_thresholding;
		threshold_value_ = config.threshold_value;
		threshold_max_value_ = config.threshold_max_value;
		threshold_type_ = config.threshold_type;
	}

	IplImage * processImage( IplImage * ipl_img )
	{
		printf( "got image\n" );

		if ( !process_image_initialized_ )
		{
			for ( unsigned int i = 0; i < distance_images_.size(); ++i )
			{
				distance_images_[i] = cvCreateImage( cvSize( ipl_img->width,
				                                             ipl_img->height ),
				                                     IPL_DEPTH_32F,
				                                     1 );
			}

			process_image_initialized_ = true;
		}

		if ( enable_meanshift_ )
		{
			cvPyrMeanShiftFiltering( ipl_img,
			                         ipl_img,
			                         ms_spatial_radius_,
			                         ms_color_radius_,
			                         ms_max_level_,
			                         cvTermCriteria( CV_TERMCRIT_ITER + CV_TERMCRIT_EPS
			                                         ,
			                                         ms_max_iter_,
			                                         ms_min_epsilon_ ) );
		}

		// desired image size: 160x120
		/*cv::resize( cv_img_,
		 output_img,
		 cv::Size(),
		 160.0 / (double) cv_img_.size().width,
		 120.0 / (double) cv_img_.size().height );*/

		cvCvtColor( ipl_img,
		            ipl_img,
		            CV_BGR2HSV );

		unsigned char * original_pixel;
		float * distance_pixel;

		// generate color distance image
		for ( unsigned int y = 0; y < ipl_img->height; y++ )
		{
			for ( unsigned int x = 0; x < ipl_img->width; x++ )
			{
				// get pixel data
				original_pixel = opencv_utils::getIplPixel<unsigned char>( ipl_img,
				                                                           x,
				                                                           y );
				_Color3f current_color( original_pixel );

				float distance_pixel_min = std::numeric_limits<float>::max();
				unsigned int distance_pixel_min_index = -1;

				// calculate the distance from the current pixel's color to all our desired colors
				for ( unsigned int color_index = 0; color_index < OutputColorRGB::NUM_COLORS; ++color_index )
				{

					distance_pixel = opencv_utils::getIplPixel<float>( distance_images_[color_index],
					                                                   x,
					                                                   y );

					static std::array<float, 3> radii = { 90.0, 0.0, 0.0 };

					*distance_pixel = 1.0 - target_colors_[color_index].color.distance( current_color,
					                                                                    radii,
					                                                                    target_colors_[color_index].weight.data_,
					                                                                    target_colors_[color_index].variance.data_,
					                                                                    DistanceType::GAUSSIAN ); //, weights );

					if ( target_colors_[color_index].enabled && *distance_pixel < distance_pixel_min && *distance_pixel < target_colors_[color_index].threshold )
					{
						distance_pixel_min = *distance_pixel;
						distance_pixel_min_index = color_index;
					}

					*distance_pixel *= 255.0;
				}

				OutputColorRGB::_CvColorType output_pixel = OutputColorRGB::getColorRGB( distance_pixel_min_index );

				original_pixel[0] = output_pixel[0];
				original_pixel[1] = output_pixel[1];
				original_pixel[2] = output_pixel[2];
			}
		}

		// go through the processed images, optionally threshold each one, and publish them
		for ( unsigned int color_index = 0; color_index < OutputColorRGB::NUM_COLORS; ++color_index )
		{
			if ( enable_thresholding_ )
			{
				cvThreshold( distance_images_[color_index],
				             distance_images_[color_index],
				             threshold_value_,
				             threshold_max_value_,
				             threshold_type_ );
			}

			//cvNormalize( distance_images_[color_index], distance_images_[color_index], 0.0, 255.0, CV_MINMAX );

			opencv_utils::publishCvImage( distance_images_[color_index],
			                              &image_transport_publishers_[color_index],
			                              &image_bridges_[color_index] );

		}

		return ipl_img;
	}

};

#endif /* COLOR_CLASSIFIER_H_ */
