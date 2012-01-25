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

/* msgs */
#include <base_libs/ComponentImageArray.h>

/* srvs */
#include <color_classifier/SetEnabledColors.h>

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
#include <common_utils/colors.h>

typedef color_classifier::ColorClassifierConfig _ReconfigureType;
typedef double _DataType;
typedef ThresholdedColor<_DataType, 3> _ThresholdedColor;

class ColorClassifier: public BaseImageProc<_ReconfigureType>
{

public:
	typedef std::array<_ThresholdedColor, OutputColorRGB::NUM_COLORS> _ThresholdedColorArrayType;
	typedef std::array<IplImage *, OutputColorRGB::NUM_COLORS> _IplImagePtrArrayType;
	typedef std::array<sensor_msgs::CvBridge, OutputColorRGB::NUM_COLORS> _CvBridgeArrayType;

	/* pubs */
	ros::Publisher images_pub_;

	/* svrs */
	ros::ServiceServer set_enabled_colors_svr_;

	/* reconfigure params */
	_ThresholdedColorArrayType target_colors_;
	_IplImagePtrArrayType distance_images_;
	_CvBridgeArrayType image_bridges_;

	/* constructor params */
	bool process_image_initialized_;
	//IplConvKernel * kernel_;

public:

	ColorClassifier( ros::NodeHandle & nh ) :
			BaseImageProc<_ReconfigureType>( nh ), process_image_initialized_( false )//, kernel_( cvCreateStructuringElementEx( 6, 6, 3, 3, CV_SHAPE_ELLIPSE ) )
	{
		images_pub_ = nh_local_.advertise<base_libs::ComponentImageArray>( "images", 1 );
		set_enabled_colors_svr_ = nh_local_.advertiseService( "set_enabled_colors", &ColorClassifier::setEnabledColorsCB, this );

		initCfgParams();
	}

	virtual ~ColorClassifier()
	{
		for ( _DimType i = 0; i < distance_images_.size(); ++ i )
		{
			if( distance_images_[i] ) cvReleaseImage( &distance_images_[i] );
		}

		//if( kernel_ ) cvReleaseStructuringElement( &kernel_ );
	}

	bool setEnabledColorsCB( color_classifier::SetEnabledColors::Request & req, color_classifier::SetEnabledColors::Response & resp )
	{

		// disable everything
		for( _DimType i = 0; i < OutputColorRGB::NUM_COLORS; ++i )
		{
			target_colors_[i].enabled = false;
		}

		// enable the colors in the request
		for( _DimType i = 0; i < req.colors.size(); ++i )
		{
			target_colors_[req.colors[i]].enabled = true;
		}

		return true;
	}

	// virtual
	void reconfigureCB( _ReconfigureType &config,
	                    uint32_t level )
	{
		//target_colors_[OutputColorRGB::Id::red].enabled = config.red_filter_enabled;
		*target_colors_[OutputColorRGB::Id::red].color.i_ = config.red_hue;
		*target_colors_[OutputColorRGB::Id::red].color.j_ = config.red_sat;
		*target_colors_[OutputColorRGB::Id::red].color.k_ = config.red_val;
		target_colors_[OutputColorRGB::Id::red].threshold = config.red_threshold;
		/**target_colors_[OutputColorRGB::Id::red].weight.i_ = config.red_hue_weight;
		*target_colors_[OutputColorRGB::Id::red].weight.j_ = config.red_sat_weight;
		*target_colors_[OutputColorRGB::Id::red].weight.k_ = config.red_val_weight;*/
		*target_colors_[OutputColorRGB::Id::red].variance.i_ = config.red_hue_variance;
		*target_colors_[OutputColorRGB::Id::red].variance.j_ = config.red_sat_variance;
		*target_colors_[OutputColorRGB::Id::red].variance.k_ = config.red_val_variance;

		//target_colors_[OutputColorRGB::Id::orange].enabled = config.orange_filter_enabled;
		*target_colors_[OutputColorRGB::Id::orange].color.i_ = config.orange_hue;
		*target_colors_[OutputColorRGB::Id::orange].color.j_ = config.orange_sat;
		*target_colors_[OutputColorRGB::Id::orange].color.k_ = config.orange_val;
		target_colors_[OutputColorRGB::Id::orange].threshold = config.orange_threshold;
		/**target_colors_[OutputColorRGB::Id::orange].weight.i_ = config.orange_hue_weight;
		*target_colors_[OutputColorRGB::Id::orange].weight.j_ = config.orange_sat_weight;
		*target_colors_[OutputColorRGB::Id::orange].weight.k_ = config.orange_val_weight;*/
		*target_colors_[OutputColorRGB::Id::orange].variance.i_ = config.orange_hue_variance;
		*target_colors_[OutputColorRGB::Id::orange].variance.j_ = config.orange_sat_variance;
		*target_colors_[OutputColorRGB::Id::orange].variance.k_ = config.orange_val_variance;

		//target_colors_[OutputColorRGB::Id::yellow].enabled = config.yellow_filter_enabled;
		*target_colors_[OutputColorRGB::Id::yellow].color.i_ = config.yellow_hue;
		*target_colors_[OutputColorRGB::Id::yellow].color.j_ = config.yellow_sat;
		*target_colors_[OutputColorRGB::Id::yellow].color.k_ = config.yellow_val;
		target_colors_[OutputColorRGB::Id::yellow].threshold = config.yellow_threshold;
		/**target_colors_[OutputColorRGB::Id::yellow].weight.i_ = config.yellow_hue_weight;
		*target_colors_[OutputColorRGB::Id::yellow].weight.j_ = config.yellow_sat_weight;
		*target_colors_[OutputColorRGB::Id::yellow].weight.k_ = config.yellow_val_weight;*/
		*target_colors_[OutputColorRGB::Id::yellow].variance.i_ = config.yellow_hue_variance;
		*target_colors_[OutputColorRGB::Id::yellow].variance.j_ = config.yellow_sat_variance;
		*target_colors_[OutputColorRGB::Id::yellow].variance.k_ = config.yellow_val_variance;

		//target_colors_[OutputColorRGB::Id::green].enabled = config.green_filter_enabled;
		*target_colors_[OutputColorRGB::Id::green].color.i_ = config.green_hue;
		*target_colors_[OutputColorRGB::Id::green].color.j_ = config.green_sat;
		*target_colors_[OutputColorRGB::Id::green].color.k_ = config.green_val;
		target_colors_[OutputColorRGB::Id::green].threshold = config.green_threshold;
		/**target_colors_[OutputColorRGB::Id::green].weight.i_ = config.green_hue_weight;
		*target_colors_[OutputColorRGB::Id::green].weight.j_ = config.green_sat_weight;
		*target_colors_[OutputColorRGB::Id::green].weight.k_ = config.green_val_weight;*/
		*target_colors_[OutputColorRGB::Id::green].variance.i_ = config.green_hue_variance;
		*target_colors_[OutputColorRGB::Id::green].variance.j_ = config.green_sat_variance;
		*target_colors_[OutputColorRGB::Id::green].variance.k_ = config.green_val_variance;

		//target_colors_[OutputColorRGB::Id::blue].enabled = config.blue_filter_enabled;
		*target_colors_[OutputColorRGB::Id::blue].color.i_ = config.blue_hue;
		*target_colors_[OutputColorRGB::Id::blue].color.j_ = config.blue_sat;
		*target_colors_[OutputColorRGB::Id::blue].color.k_ = config.blue_val;
		target_colors_[OutputColorRGB::Id::blue].threshold = config.blue_threshold;
		/**target_colors_[OutputColorRGB::Id::blue].weight.i_ = config.blue_hue_weight;
		*target_colors_[OutputColorRGB::Id::blue].weight.j_ = config.blue_sat_weight;
		*target_colors_[OutputColorRGB::Id::blue].weight.k_ = config.blue_val_weight;*/
		*target_colors_[OutputColorRGB::Id::blue].variance.i_ = config.blue_hue_variance;
		*target_colors_[OutputColorRGB::Id::blue].variance.j_ = config.blue_sat_variance;
		*target_colors_[OutputColorRGB::Id::blue].variance.k_ = config.blue_val_variance;

		//target_colors_[OutputColorRGB::Id::black].enabled = config.black_filter_enabled;
		*target_colors_[OutputColorRGB::Id::black].color.i_ = config.black_hue;
		*target_colors_[OutputColorRGB::Id::black].color.j_ = config.black_sat;
		*target_colors_[OutputColorRGB::Id::black].color.k_ = config.black_val;
		target_colors_[OutputColorRGB::Id::black].threshold = config.black_threshold;
		/**target_colors_[OutputColorRGB::Id::black].weight.i_ = config.black_hue_weight;
		*target_colors_[OutputColorRGB::Id::black].weight.j_ = config.black_sat_weight;
		*target_colors_[OutputColorRGB::Id::black].weight.k_ = config.black_val_weight;*/
		*target_colors_[OutputColorRGB::Id::black].variance.i_ = config.black_hue_variance;
		*target_colors_[OutputColorRGB::Id::black].variance.j_ = config.black_sat_variance;
		*target_colors_[OutputColorRGB::Id::black].variance.k_ = config.black_val_variance;

		//target_colors_[OutputColorRGB::Id::white].enabled = config.white_filter_enabled;
		*target_colors_[OutputColorRGB::Id::white].color.i_ = config.white_hue;
		*target_colors_[OutputColorRGB::Id::white].color.j_ = config.white_sat;
		*target_colors_[OutputColorRGB::Id::white].color.k_ = config.white_val;
		target_colors_[OutputColorRGB::Id::white].threshold = config.white_threshold;
		/**target_colors_[OutputColorRGB::Id::white].weight.i_ = config.white_hue_weight;
		*target_colors_[OutputColorRGB::Id::white].weight.j_ = config.white_sat_weight;
		*target_colors_[OutputColorRGB::Id::white].weight.k_ = config.white_val_weight;*/
		*target_colors_[OutputColorRGB::Id::white].variance.i_ = config.white_hue_variance;
		*target_colors_[OutputColorRGB::Id::white].variance.j_ = config.white_sat_variance;
		*target_colors_[OutputColorRGB::Id::white].variance.k_ = config.white_val_variance;
	}

	IplImage * processImage( IplImage * ipl_img )
	{
		printf( "got image\n" );

		if ( !process_image_initialized_ )
		{
			for ( _DimType i = 0; i < distance_images_.size(); ++ i )
			{
				distance_images_[i] = cvCreateImage( cvSize( ipl_img->width,
				                                             ipl_img->height ),
				                                     IPL_DEPTH_8U,
				                                     1 );
			}

			process_image_initialized_ = true;
		}

		bool some_color_enabled = false;
		for ( unsigned int i = 0; i < OutputColorRGB::NUM_COLORS; ++i )
		{
			if( target_colors_[i].enabled )
			{
				some_color_enabled = true;
				break;
			}
		}
		if( !some_color_enabled ) return ipl_img;

		if ( reconfigure_params_.enable_meanshift )
		{
			cvPyrMeanShiftFiltering( ipl_img,
			                         ipl_img,
			                         reconfigure_params_.ms_spatial_radius,
			                         reconfigure_params_.ms_color_radius,
			                         reconfigure_params_.ms_max_level,
			                         cvTermCriteria( CV_TERMCRIT_ITER + CV_TERMCRIT_EPS
			                                         ,
			                                         reconfigure_params_.ms_max_iter,
			                                         reconfigure_params_.ms_min_epsilon ) );
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
		unsigned char * probability_pixel;
		std::array<_DataType, 3> radii = { 90.0, 0.0, 0.0 };
		std::array<_DataType, 3> weights = { 1.0, 1.0, 1.0 };

		// generate color distance image
		for ( unsigned int y = 0; y < ipl_img->height; y++ )
		{
			for ( unsigned int x = 0; x < ipl_img->width; x++ )
			{
				// get pixel data
				original_pixel = opencv_utils::getIplPixel<unsigned char>( ipl_img,
				                                                           x,
				                                                           y );
				Color<_DataType, 3> current_color( original_pixel );

				_DataType probability_pixel_max = std::numeric_limits<_DataType>::min();
				unsigned int probability_pixel_max_index = -1;

				// calculate the distance from the current pixel's color to all our desired colors
				for ( unsigned int color_index = 0; color_index < OutputColorRGB::NUM_COLORS; ++color_index )
				{
					// skip this image if it's not enabled
					if( !target_colors_[color_index].enabled ) continue;

					probability_pixel = opencv_utils::getIplPixel<unsigned char>( distance_images_[color_index],
					                                                       x,
					                                                       y );

					_DataType probability = target_colors_[color_index].color.distance( current_color,
						                                                                 radii,
						                                                              	 weights,
						                                                              	 target_colors_[color_index].variance.data_,
						                                                              	 DistanceType::GAUSSIAN );

					*probability_pixel = probability > target_colors_[color_index].threshold ? 255 : 0;

					if ( target_colors_[color_index].enabled && probability > probability_pixel_max && *probability_pixel )
					{
						probability_pixel_max = probability;
						probability_pixel_max_index = color_index;
					}

					//*distance_pixel *= 255.0;
				}

				OutputColorRGB::_CvColorType output_pixel = OutputColorRGB::getColorRGB( probability_pixel_max_index );

				original_pixel[0] = output_pixel[0];
				original_pixel[1] = output_pixel[1];
				original_pixel[2] = output_pixel[2];
			}
		}

		base_libs::ComponentImageArray::Ptr component_image_array_message( new base_libs::ComponentImageArray );
		component_image_array_message->header = image_msg_->header;

		// go through the processed images, optionally threshold each one, and publish them
		for ( unsigned int color_index = 0; color_index < OutputColorRGB::NUM_COLORS; ++color_index )
		{
			// skip this image if it's not enabled
			if( !target_colors_[color_index].enabled ) continue;

			IplImage * current_image = distance_images_[color_index];

			cvMorphologyEx( current_image, current_image, 0, 0, CV_MOP_OPEN, reconfigure_params_.open_iterations );
			cvMorphologyEx( current_image, current_image, 0, 0, CV_MOP_CLOSE, reconfigure_params_.close_iterations );


			// erode the image to remove noise
			//cvErode( current_image, current_image, kernel_, reconfigure_params_.open_iterations );

			// dilate the image to restore blobs to approximately their original size
			//cvDilate( current_image, current_image, kernel_, reconfigure_params_.close_iterations );

			const sensor_msgs::Image::ConstPtr & image = sensor_msgs::CvBridge::cvToImgMsg( current_image );

			base_libs::ComponentImage component_image_msg;
			component_image_msg.image = *image;
			component_image_msg.image.header = image_msg_->header;
			//component_image_msg.image.header = image_msg_->header;
			component_image_msg.id = color_index;

			component_image_array_message->images.push_back( component_image_msg );
		}

		images_pub_.publish( component_image_array_message );

		return ipl_img;
		//return distance_images_[1];
	}

};

#endif /* COLOR_CLASSIFIER_H_ */
