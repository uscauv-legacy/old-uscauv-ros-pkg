/***************************************************************************
 *  include/color_classifier/color_classifier_trainer_node.h
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

#ifndef COLORCLASSIFIER_COLORCLASSIFIERTRAINERNODE_H_
#define COLORCLASSIFIER_COLORCLASSIFIERTRAINERNODE_H_

#include <quickdev/node.h>

#include <quickdev/gaussian_pdf.h>
#include <quickdev/param_reader.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>
//#include <opencv/imgproc.h>

QUICKDEV_DECLARE_NODE( ColorClassifierTrainer )

QUICKDEV_DECLARE_NODE_CLASS( ColorClassifierTrainer )
{
    QUICKDEV_DECLARE_NODE_CONSTRUCTOR( ColorClassifierTrainer )
    {
        //
    }

    QUICKDEV_SPIN_FIRST()
    {
        QUICKDEV_GET_RUNABLE_NODEHANDLE( nh_rel );

        typedef quickdev::GaussianPDF<3> _ColorModel;
        typedef _ColorModel::_DataPoint _DataPoint;

        auto src_image_uri = ros::ParamReader<std::string, 1>::readParam( nh_rel, "src_image_uri", "" );
        auto mask_image_uri = ros::ParamReader<std::string, 1>::readParam( nh_rel, "mask_image_uri", "" );
        auto color_name = ros::ParamReader<std::string, 1>::readParam( nh_rel, "color_name", "" );

        initPolicies<quickdev::policy::ALL>();

        // load color
        cv::Mat const input_image = cv::imread( src_image_uri );
        // load grayscale
        cv::Mat const mask_image = cv::imread( mask_image_uri, 0 );

        if( input_image.empty() )
        {
            PRINT_ERROR( "Input image is empty; exiting" );
            exit( 1 );
        }

        if( mask_image.empty() )
        {
            PRINT_ERROR( "Mask image is empty; exiting" );
            exit( 1 );
        }

        // convert to target color space
        cv::Mat input_image_lab;
        cv::cvtColor( input_image, input_image_lab, CV_BGR2HLS );

        // build up our color model
        _ColorModel color_model_;

        for( int y = 0; y < mask_image.size().height; y++ )
        {
            for( int x = 0; x < mask_image.size().width; x++ )
            {
                auto const & mask_pixel = mask_image.at<unsigned char>( y, x );

                // ignore "black" mask pixels
                if( mask_pixel <= 255 / 2 ) continue;

                auto const & lab_pixel = input_image_lab.at<cv::Vec3b>( y, x );
                color_model_.push_back( _DataPoint( (double)lab_pixel[0], (double)lab_pixel[1], (double)lab_pixel[2] ) );
            }
        }

        std::cout << "Sampling from " << color_model_.size() << " data points." << std::endl;

        auto const & mean = color_model_.updateMean();
        auto const & covariance = color_model_.updateCovariance();

        std::cout << mean << std::endl;

        std::cout << covariance << std::endl;

        // Output mean and variance (not full covariance)
        XmlRpc::XmlRpcValue mean_param;
        XmlRpc::XmlRpcValue cov_param;

        for( size_t i = 0; i < mean.size(); ++i )
        {
            mean_param[i] = mean[i];
            cov_param[i] = covariance( i, i );
        }

        nh_rel.setParam( "model/" + color_name + "/mean", mean_param );

        nh_rel.setParam( "model/" + color_name + "/cov", cov_param );

/* Output full covariance
        for( size_t i = 0; i < covariance.size(); ++i )
        {
            std::stringstream index_ss;
            index_ss << i;
            auto const index_str = index_ss.str();

            nh_rel.setParam( "model/" + color_name + "/cov/elem" + index_str, covariance[i] );
        }
*/
        auto const dump_result = system( std::string( "rosparam dump `rospack find color_classifier`/params/model.yaml /color_classifier_trainer/model/" ).c_str() );
    }

    QUICKDEV_SPIN_ONCE()
    {
        interrupt();
    }
};

#endif // COLORCLASSIFIER_COLORCLASSIFIERTRAINERNODE_H_
