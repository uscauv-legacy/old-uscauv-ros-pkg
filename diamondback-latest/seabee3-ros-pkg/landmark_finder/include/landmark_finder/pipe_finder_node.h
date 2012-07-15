/***************************************************************************
 *  include/landmark_finder/pipe_finder_node.h
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

#ifndef LANDMARKFINDER_PIPEFINDERNODE_H_
#define LANDMARKFINDER_PIPEFINDERNODE_H_

#include <quickdev/node.h>

#include <quickdev/multi_subscriber.h>

QUICKDEV_DECLARE_NODE( PipeFinder )

QUICKDEV_DECLARE_NODE_CLASS( PipeFinder )
{
    ros::MultiSubscriber<> multi_sub_;

    XmlRpc::XmlRpcValue params_;

    QUICKDEV_DECLARE_NODE_CONSTRUCTOR( PipeFinder )
    {
        //
    }

    QUICKDEV_SPIN_FIRST()
    {
        initPolicies<quickdev::policy::ALL>();

        QUICKDEV_GET_RUNABLE_NODEHANDLE( nh_rel );

        params_ = quickdev::ParamReader::readParam<decltype( params_ )>( nh_rel, "params" );
    }

/*
    QUICKDEV_DECLARE_MESSAGE_CALLBACK( imagesCB, _NamedImageArrayMsg )
    {

        for( auto named_image_msg = msg->images.cbegin(); named_image_msg != msg->images.cend(); ++named_image_msg )
        {
            if( named_image_msg->name != "orange" ) continue;

            auto const & aspect_ratio_mean = double( params_["aspect_ratio"]["mean"] );
            auto const & aspect_ratio_variance = double( params_["aspect_ratio"]["variance"] );

            auto const cv_image_ptr = quickdev::opencv_conversion::fromImageMsg( named_image_msg->image );
            cv::Mat const & image = cv_image_ptr->image;

            cv::Mat debug_image( image.size(), CV_8UC3 );


            const float ASPECT_RATIO_BOUNDARY = 1.5;
            const float perimScale = 10;
            //cv::Mat input = image_ptr->image;
            cv::Mat gray_input;
            std::vector<CvBox2D> usable_boxes;
            CvMemStorage* storage;
            CvSeq* contours;
            CvBox2D box_to_check;
            storage = cvCreateMemStorage(0);
            contours = cvCreateSeq(
                                    CV_SEQ_ELTYPE_POINT,
                                    sizeof( CvSeq ),
                                    sizeof( CvPoint ),
                                    storage
                                  );
            //make 8uc1 Mat
            //cvtColor(input, gray_input, CV_RGB2GRAY);
            threshold(input, gray_input, 127, 255, CV_THRESH_BINARY);


            cv::Mat elem = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5,5));
            cv::morphologyEx(gray_input, gray_input, cv::MORPH_OPEN, elem, cv::Point(-1,-1), 3);

            IplImage gray_input_ipl( gray_input );

            _ImageProcPolicy::publishImages( "output_image", quickdev::opencv_conversion::fromMat( image ) );
        }
    }
*/

    QUICKDEV_SPIN_ONCE()
    {
        //
    }
};

#endif // LANDMARKFINDER_PIPEFINDERNODE_H_
