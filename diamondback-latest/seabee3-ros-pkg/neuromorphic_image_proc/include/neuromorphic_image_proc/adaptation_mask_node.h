/***************************************************************************
 *  include/neuromorphic_image_proc/adaptation_mask_node.h
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

#ifndef NEUROMORPHICIMAGEPROC_ADAPTATIONMASK_H_
#define NEUROMORPHICIMAGEPROC_ADAPTATIONMASK_H_

#include <quickdev/node.h>
#include <neuromorphic_image_proc/adaptation_image_proc_policy.h>
#include <quickdev/feature.h>

QUICKDEV_DECLARE_NODE( AdaptationMask, AdaptationImageProcPolicy )

QUICKDEV_DECLARE_NODE_CLASS( AdaptationMask )
{
    cv::Mat last_image_;
    cv::Mat adaptation_image_;
    cv::Mat high_values_mask_;
    cv::Mat adaptation_mask_;
    cv::Mat adaptation_time_image_;
    cv::Mat adaptation_mask_image_;
    const static unsigned int threshold_ = 30;

    QUICKDEV_DECLARE_NODE_CONSTRUCTOR( AdaptationMask )
    {
        //
    }

    QUICKDEV_SPIN_FIRST()
    {
        initPolicies<quickdev::policy::ALL>();
    }

    IMAGE_PROC_PROCESS_IMAGE( image_ptr )
    {
        //IplImage * image = &IplImage( image_ptr->image );

        cv::Mat const & image = image_ptr->image;
        //cv::Mat lab_image;
        cv::Mat hsl_image;
        //cv::cvtColor( image, lab_image, CV_BGR2Lab );
        cv::cvtColor( image, hsl_image, CV_BGR2HLS );

        cv::GaussianBlur( hsl_image, hsl_image, cv::Size( 3, 3 ), 0 );

        // convert our LAB image to float
        //cv::Mat lab_image_float;
        //lab_image.convertTo( lab_image_float, CV_32F );

        // find the difference for each pixel (in time)
        if( last_image_.empty() ) last_image_ = cv::Mat( hsl_image.rows, hsl_image.cols, CV_8UC3 );
        cv::absdiff( hsl_image, last_image_, adaptation_image_ );

        // flatten the image (sum the channels)
        std::vector<cv::Mat> adaptation_images( 3 );
        cv::split( adaptation_image_, adaptation_images );

        if( adaptation_time_image_.empty() ) adaptation_time_image_ = cv::Mat( adaptation_mask_image_.rows, adaptation_mask_image_.cols, CV_8UC1 );
        adaptation_time_image_ += 1;
        adaptation_mask_image_ = adaptation_images[0] * 0.33 + adaptation_images[1] * 0.33 + adaptation_images[2] * 0.33 + adaptation_time_image_;

        cv::threshold( adaptation_mask_image_, high_values_mask_, threshold_, 255, CV_THRESH_BINARY );

        //cv::GaussianBlur( high_values_mask_, high_values_mask_, cv::Size( 3, 3 ), 0 );
        //cv::threshold( high_values_mask_, high_values_mask_, 127, 255, CV_THRESH_BINARY );

        publishImages( "output_image", quickdev::opencv_conversion::fromMat( hsl_image ) );
        publishImages( "output_adaptation_mask", quickdev::opencv_conversion::fromMat( high_values_mask_, "", "mono8" ) );

        hsl_image.copyTo( last_image_, high_values_mask_ );
        adaptation_time_image_.setTo( cv::Scalar( 0 ), high_values_mask_ );
    }

    QUICKDEV_SPIN_ONCE()
    {
        //
    }
};

#endif // NEUROMORPHICIMAGEPROC_ADAPTATIONMASK_H_
