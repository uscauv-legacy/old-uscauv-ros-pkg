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
#include <seabee3_common/NamedImageArray.h>
#include <seabee3_common/colors.h>
#include <limits>

QUICKDEV_DECLARE_NODE( ColorClassifier, AdaptationImageProcPolicy )

QUICKDEV_DECLARE_NODE_CLASS( ColorClassifier )
{
protected:
    typedef seabee3_common::NamedImageArray _NamedImageArrayMsg;

    std::vector<cv::Mat> classified_images_;
    ros::MultiPublisher<> multi_pub_;

    QUICKDEV_DECLARE_NODE_CONSTRUCTOR( ColorClassifier ),
        classified_images_( seabee3_common::colors::PIXELS.size() )
    {
        //
    }

    QUICKDEV_SPIN_FIRST()
    {
        initPolicies<quickdev::policy::ALL>();

        QUICKDEV_GET_RUNABLE_NODEHANDLE( nh_rel );

        multi_pub_.addPublishers<_NamedImageArrayMsg>( nh_rel, {"classified_images"} );
    }

    IMAGE_PROC_PROCESS_IMAGE( image_ptr )
    {
        cv::Mat const & image = image_ptr->image;
        cv::Mat const & mask = adaptation_mask_ptr_->image;

        cv::Mat_<cv::Vec3b>::const_iterator current_image_pixel = image.begin<cv::Vec3b>();
        cv::Mat_<uchar>::const_iterator current_mask_pixel = mask.begin<uchar>();

        for( size_t x = 0; x < (size_t)image.cols; ++x )
        {
            for( size_t y = 0; y < (size_t)image.rows; ++y )
            {
                if( !mask.empty() )
                {
                    auto const & mask_pixel = mask.at<uchar>( y, x );
                    if( !mask_pixel ) continue;
                }

                auto const & raw_pixel = image.at<cv::Vec3b>( y, x );
                auto const & pixel = quickdev::pixel::make_pixel( raw_pixel );

                for( size_t i = 0; i < classified_images_.size(); ++i )
                {
                    if( false ) continue;
                    auto const & desired_color = seabee3_common::colors::PIXELS[i];
                    quickdev::Feature<float> const sigma( 0.1f, 0.1f, 0.1f );
                    classified_images_[i].at<uchar>( y, x ) = std::numeric_limits<uchar>::max() * pixel.distanceTo<quickdev::feature::mode::distance::GAUSSIAN>( desired_color, sigma, 1.0 );
                }
            }
        }

        _NamedImageArrayMsg named_image_array_message;
        named_image_array_message.images.resize( classified_images_.size() );

        for( size_t i = 0; i < classified_images_.size(); ++i )
        {
            named_image_array_message.images[i].image = *quickdev::opencv_conversion::fromMat( classified_images_[i] );
        }

        multi_pub_.publish( "classified_images", named_image_array_message );

        publishImages( "output_adaptation_image", quickdev::opencv_conversion::fromMat( mask ) );

    }

    QUICKDEV_SPIN_ONCE()
    {
        //
    }
};

#endif // COLORCLASSIFIER_COLORCLASSIFIERNODE_H_
