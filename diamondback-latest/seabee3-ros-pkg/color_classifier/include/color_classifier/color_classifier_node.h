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

// policies
#include <neuromorphic_image_proc/adaptation_image_proc_policy.h>
#include <quickdev/action_server_policy.h>

// objects
#include <quickdev/pixel.h>
#include <quickdev/param_reader.h>
#include <set>

// utils
#include <limits>

// actions
#include <seabee3_actions/ConfigureAction.h>

// msgs
#include <seabee3_msgs/NamedImageArray.h>

typedef seabee3_msgs::NamedImageArray _NamedImageArrayMsg;

typedef seabee3_actions::ConfigureAction _ConfigureAction;

typedef quickdev::ActionServerPolicy<_ConfigureAction> _ConfigureActionServerPolicy;
typedef AdaptationImageProcPolicy _AdaptationImageProcPolicy;

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

QUICKDEV_DECLARE_NODE( ColorClassifier, _AdaptationImageProcPolicy, _ConfigureActionServerPolicy )

QUICKDEV_DECLARE_NODE_CLASS( ColorClassifier )
{
protected:
//    typedef quickdev::ImageProcPolicy _ImageProcPolicy;
    typedef ClassifiedColor _ClassifiedColor;
    typedef _ClassifiedColor::_Mean _ColorMean;
    typedef _ClassifiedColor::_Covariance _ColorCovariance;

    std::set<std::string> color_filter_;
    std::mutex color_filter_mutex_;

    std::map<std::string, cv::Mat> classified_images_;
    ros::MultiPublisher<> multi_pub_;

    std::map<std::string, _ClassifiedColor> target_colors_;

    XmlRpc::XmlRpcValue model_;

    QUICKDEV_DECLARE_NODE_CONSTRUCTOR( ColorClassifier )
    {
        //
    }

    QUICKDEV_SPIN_FIRST()
    {
        QUICKDEV_GET_RUNABLE_NODEHANDLE( nh_rel );

        _ConfigureActionServerPolicy::registerExecuteCB( quickdev::auto_bind( &ColorClassifierNode::configureActionExecuteCB, this ) );

        initPolicies<_ConfigureActionServerPolicy>( "action_name_param", std::string( "set_color_filter" ) );

        _AdaptationImageProcPolicy::registerCombinedImageCallback( quickdev::auto_bind( &ColorClassifierNode::imagesCB, this ) );

//        _ImageProcPolicy::addImagePublisher( "classified_images" );

        multi_pub_.addPublishers<_NamedImageArrayMsg>( nh_rel, {"classified_images"} );

        model_ = quickdev::ParamReader::readParam<decltype( model_ ) >( nh_rel, "model" );

        for( auto color_it = model_.begin(); color_it != model_.end(); ++color_it )
        {
            auto const & color_name = color_it->first;
            std::cout << "Loading settings for color " << color_name << std::endl;
            color_filter_.insert( color_name );

            auto & color = color_it->second;

            auto const color_mean = quickdev::ParamReader::getXmlRpcValue<std::vector<double> >( color, "mean" );
            auto const color_cov = quickdev::ParamReader::getXmlRpcValue<std::vector<double> >( color, "cov" );

            _ClassifiedColor const classified_color( color_mean, color_cov );

            std::cout << " - mean: " << classified_color.mean_ << std::endl;
            std::cout << " - cov: " << classified_color.covariance_ << std::endl;

            target_colors_[color_name] = classified_color;
            classified_images_[color_name] = cv::Mat();
        }

        initPolicies<quickdev::policy::ALL>();
    }

    QUICKDEV_DECLARE_ACTION_EXECUTE_CALLBACK( configureActionExecuteCB, _ConfigureAction )
    {
        // lock color_filter mutex
        auto lock = quickdev::make_unique_lock( color_filter_mutex_ );

        auto const & settings = goal->settings;

        // update the color_filter
        for( auto setting_it = settings.cbegin(); setting_it != settings.cend(); ++setting_it )
        {
            auto const & setting = *setting_it;

            if( setting.empty() ) continue;

            // reset the color_filter
            if( setting == "-all" ) color_filter_.clear();
            // remove the given item; -<item>
            else if( setting.substr( 0, 1 ) == "-" ) color_filter_.erase( setting.substr( 1 ) );
            // add the given item; +<item>
            else if( setting.substr( 0, 1 ) == "+" ) color_filter_.insert( setting.substr( 1 ) );
            // default; add the given item <item>
            else color_filter_.insert( setting );
        }
    }

    void imagesCB( cv_bridge::CvImageConstPtr const & image_msg, cv_bridge::CvImageConstPtr const & mask_msg )
    {
//        std::cout << "Getting image from message" << std::endl;
        if( !image_msg )
        {
            PRINT_ERROR( "image message is null" );
            return;
        }
        cv::Mat const & image = image_msg->image;
        cv::Mat normalized_image;
        cv::GaussianBlur( image, normalized_image, cv::Size( 15, 15 ), 1 );
//        std::cout << "Getting mask from message" << std::endl;
        if( !mask_msg )
        {
            PRINT_ERROR( "mask message is null" );
            return;
        }
        cv::Mat const & mask = mask_msg->image;

//        cv::Mat_<cv::Vec3b>::const_iterator current_image_pixel = image.begin<cv::Vec3b>();
//        cv::Mat_<uchar>::const_iterator current_mask_pixel = mask.begin<uchar>();

//        std::map<std::string, size_t> num_pixels_processed_map;

//        for( size_t x = 610; x < 616; ++x )
        quickdev::make_unique_lock( color_filter_mutex_ );

        size_t const & img_width = image.size().width;
        size_t const & img_height = image.size().height;
        for( size_t x = 0; x < img_width; ++x )
        {
//            for( size_t y = 209; y < 215; ++y )
            for( size_t y = 0; y < img_height; ++y )
            {
//                PRINT_INFO( "Checking mask %zu %zu", x, y );
                auto const & mask_pixel = mask.at<uchar>( y, x );
                if( !mask_pixel ) continue;

                auto const & raw_pixel = normalized_image.at<cv::Vec3b>( y, x );
                auto const & pixel = quickdev::pixel::make_pixel<float>( raw_pixel );

//                STREAM_INFO( pixel );

                auto classified_image_it = classified_images_.begin();
                for( auto target_color_it = target_colors_.cbegin(); target_color_it != target_colors_.cend(); ++target_color_it, ++classified_image_it )
                {
                    auto const & target_color_name = target_color_it->first;
                    if( color_filter_.count( target_color_name ) == 0 ) continue;

                    auto const & target_color = target_color_it->second;
                    auto & classified_image = classified_image_it->second;

                    if( classified_image.size() != image.size() ) classified_image.create( image.size(), CV_8UC1 );

                    //auto const match_quality = 1.0 - 50 * pixel.distanceTo<quickdev::feature::mode::distance::GAUSSIAN>( target_color.mean_, target_color.covariance_, 50.0 );
                    auto const match_quality = 1.0 - pixel.distanceTo<quickdev::feature::mode::distance::GAUSSIAN_FAST>( target_color.mean_, target_color.covariance_, 0.5 );
                    classified_image.at<uchar>( y, x ) = std::numeric_limits<uchar>::max() * match_quality;

//                    PRINT_INFO( "%zu %zu %f %u", x, y, match_quality, classified_image.at<uchar>( y, x ) );

//                    if( num_pixels_processed_map.find( target_color_name ) == num_pixels_processed_map.end() ) num_pixels_processed_map[target_color_name] = 0;

//                    num_pixels_processed_map[target_color_name] ++;
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
/*
        for( auto pixel_stats_it = num_pixels_processed_map.cbegin(); pixel_stats_it != num_pixels_processed_map.cend(); ++pixel_stats_it )
        {
            PRINT_INFO( "Processed %f%% of candidate %s pixels", 100 * (float)pixel_stats_it->second / ( image.size().width * image.size().height ), pixel_stats_it->first.c_str() );
        }
*/
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

        //publishImages( "output_adaptation_mask", quickdev::opencv_conversion::fromMat( mask, "", "mono8" ) );

    }

    QUICKDEV_SPIN_ONCE()
    {
        //
    }
};

#endif // COLORCLASSIFIER_COLORCLASSIFIERNODE_H_
