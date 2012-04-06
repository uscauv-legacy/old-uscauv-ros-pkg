/***************************************************************************
 *  include/color_classifier/color_classifier_node.h
 *  --------------------
 *
 *  Copyright (c) 2011, Edward T. Kaszubski ( ekaszubski@gmail.com )
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are
 *  met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following disclaimer
 *    in the documentation and/or other materials provided with the
 *    distribution.
 *  * Neither the name of seabee3-ros-pkg nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 **************************************************************************/

#ifndef COLORCLASSIFIER_COLORCLASSIFIERNODE_H_
#define COLORCLASSIFIER_COLORCLASSIFIERNODE_H_

#include <quickdev/node.h>
#include <neuromorphic_image_proc/adaptation_image_proc_policy.h>

#include <quickdev/pixel.h>
#include <quickdev/param_reader.h>
#include <limits>

#include <seabee3_common/NamedImageArray.h>

struct ClassifiedColor
{
    typedef quickdev::Pixel<float> _Mean;
    typedef quickdev::Feature<float> _Covariance;

    _Mean mean_;
    _Covariance covariance_;

    ClassifiedColor(){}

    ClassifiedColor( _Mean const & mean, _Covariance const & covariance )
    :
        mean_( mean ),
        covariance_( covariance )
    {
        //
    }
};

QUICKDEV_DECLARE_NODE( ColorClassifier, AdaptationImageProcPolicy )

QUICKDEV_DECLARE_NODE_CLASS( ColorClassifier )
{
protected:
    typedef seabee3_common::NamedImageArray _NamedImageArrayMsg;
    typedef ClassifiedColor _ClassifiedColor;
    typedef _ClassifiedColor::_Mean _ColorMean;
    typedef _ClassifiedColor::_Covariance _ColorCovariance;

    std::map<std::string, bool> enabled_colors_;
    std::map<std::string, cv::Mat> classified_images_;
    ros::MultiPublisher<> multi_pub_;

    std::map<std::string, _ClassifiedColor> target_colors_;

    QUICKDEV_DECLARE_NODE_CONSTRUCTOR( ColorClassifier )
    {
        //
    }

    QUICKDEV_SPIN_FIRST()
    {
        initPolicies<quickdev::policy::ALL>();

        QUICKDEV_GET_RUNABLE_NODEHANDLE( nh_rel );

        multi_pub_.addPublishers<_NamedImageArrayMsg>( nh_rel, {"classified_images"} );

        std::vector<std::string> const known_color_names = { "orange" };//ros::ParamReader<std::string, 0>::readParams( nh_rel, "color", "_name", 0 );
        enabled_colors_["orange"] = true;

        for( auto color_name_it = known_color_names.cbegin(); color_name_it != known_color_names.cend(); ++color_name_it )
        {
            auto const & color_name = *color_name_it;
            auto const color_mean = ros::ParamReader<double, 0>::readParams( nh_rel, "model/" + color_name + "/mean/elem", "", 0 );
            auto const color_cov = ros::ParamReader<double, 0>::readParams( nh_rel, "model/" + color_name + "/cov/elem", "", 0 );

            _ClassifiedColor const classified_color( color_mean, color_cov );

            std::cout << "Loaded settings for color " << color_name << std::endl;
            std::cout << " - mean: " << classified_color.mean_ << std::endl;
            std::cout << " - cov: " << classified_color.covariance_ << std::endl;

            target_colors_[color_name] = classified_color;
            classified_images_[color_name] = cv::Mat();
        }
    }

    IMAGE_PROC_PROCESS_IMAGE( image_ptr )
    {
        std::cout.precision( 5 );
        cv::Mat const & image = image_ptr->image;
        cv::Mat const & mask = adaptation_mask_ptr_ ? adaptation_mask_ptr_->image : cv::Mat();

//        cv::Mat_<cv::Vec3b>::const_iterator current_image_pixel = image.begin<cv::Vec3b>();
//        cv::Mat_<uchar>::const_iterator current_mask_pixel = mask.begin<uchar>();

        auto const have_mask = !mask.empty();

        if( !have_mask ) PRINT_WARN( "No mask available; processing entire image" );

        std::map<std::string, size_t> num_pixels_processed_map;

//        for( size_t x = 610; x < 616; ++x )
        for( size_t x = 0; x < (size_t)image.size().width; ++x )
        {
//            for( size_t y = 209; y < 215; ++y )
            for( size_t y = 0; y < (size_t)image.size().height; ++y )
            {
//                PRINT_INFO( "Checking mask %zu %zu", x, y );
                if( have_mask )
                {
                    auto const & mask_pixel = mask.at<uchar>( y, x );
                    if( !mask_pixel ) continue;
                }

                auto const & raw_pixel = image.at<cv::Vec3b>( y, x );
                auto const & pixel = quickdev::pixel::make_pixel<float>( raw_pixel );

//                STREAM_INFO( pixel );

                auto classified_image_it = classified_images_.begin();
                for( auto target_color_it = target_colors_.cbegin(); target_color_it != target_colors_.cend(); ++target_color_it, ++classified_image_it )
                {
                    auto const & target_color_name = target_color_it->first;
                    auto const & color_is_enabled_it = enabled_colors_.find( target_color_name );
                    if( color_is_enabled_it == enabled_colors_.end() || !color_is_enabled_it->second ) continue;

                    auto const & target_color = target_color_it->second;
                    auto & classified_image = classified_image_it->second;

                    if( classified_image.size() != image.size() ) classified_image.create( image.size(), CV_8UC1 );

                    //auto const match_quality = 1.0 - 50 * pixel.distanceTo<quickdev::feature::mode::distance::GAUSSIAN>( target_color.mean_, target_color.covariance_, 50.0 );
                    auto const match_quality = 1.0 - pixel.distanceTo<quickdev::feature::mode::distance::GAUSSIAN_FAST>( target_color.mean_, target_color.covariance_, 0.5 );
                    classified_image.at<uchar>( y, x ) = std::numeric_limits<uchar>::max() * match_quality;

//                    PRINT_INFO( "%zu %zu %f %u", x, y, match_quality, classified_image.at<uchar>( y, x ) );

                    if( num_pixels_processed_map.find( target_color_name ) == num_pixels_processed_map.end() ) num_pixels_processed_map[target_color_name] = 0;

                    num_pixels_processed_map[target_color_name] ++;
                }
/*
                for( size_t i = 0; i < classified_images_.size(); ++i )
                {
                    if( false ) continue;
                    auto const & desired_color = seabee3_common::colors::PIXELS[i];
                    quickdev::Feature<float> const sigma( 0.1f, 0.1f, 0.1f );
                    classified_images_[i].at<uchar>( y, x ) = std::numeric_limits<uchar>::max() * pixel.distanceTo<quickdev::feature::mode::distance::GAUSSIAN>( desired_color, sigma, 1.0 );
                }
*/
            }
        }

        for( auto pixel_stats_it = num_pixels_processed_map.cbegin(); pixel_stats_it != num_pixels_processed_map.cend(); ++pixel_stats_it )
        {
            PRINT_INFO( "Processed %f%% of candidate %s pixels", 100 * (float)pixel_stats_it->second / ( image.size().width * image.size().height ), pixel_stats_it->first.c_str() );
        }

        _NamedImageArrayMsg named_image_array_message;
        named_image_array_message.images.resize( classified_images_.size() );

        size_t i = 0;
        for( auto classified_image_it = classified_images_.cbegin(); classified_image_it != classified_images_.cend(); ++classified_image_it, ++i )
        {
            auto const & color_name = classified_image_it->first;
            auto const & classified_image = classified_image_it->second;

            named_image_array_message.images[i].name = color_name;
            named_image_array_message.images[i].image = *quickdev::opencv_conversion::fromMat( classified_image, "", "mono8" );
        }

        multi_pub_.publish( "classified_images", named_image_array_message );

        publishImages( "output_adaptation_mask", quickdev::opencv_conversion::fromMat( mask, "", "mono8" ) );

    }

    QUICKDEV_SPIN_ONCE()
    {
        //
    }
};

#endif // COLORCLASSIFIER_COLORCLASSIFIERNODE_H_
