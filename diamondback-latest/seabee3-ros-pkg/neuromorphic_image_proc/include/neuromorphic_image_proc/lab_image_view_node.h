/***************************************************************************
 *  include/neuromorphic_image_proc/lab_image_view_node.h
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

#ifndef NEUROMORPHICIMAGEPROC_LABIMAGEVIEW_H_
#define NEUROMORPHICIMAGEPROC_LABIMAGEVIEW_H_

#include <quickdev/node.h>
#include <quickdev/image_proc_policy.h>

QUICKDEV_DECLARE_NODE( LabImageView, quickdev::ImageProcPolicy )

QUICKDEV_DECLARE_NODE_CLASS( LabImageView )
{
    QUICKDEV_DECLARE_NODE_CONSTRUCTOR( LabImageView )
    {
        cv::namedWindow( "L_a_b", 0 );
    }

    QUICKDEV_SPIN_FIRST()
    {
        initAll();
    }

    IMAGE_PROC_PROCESS_IMAGE( image_ptr )
    {
        cv::Mat const & image = image_ptr->image;

        std::vector<cv::Mat> image_channels( 3 );

        // split 3-channel image into 3 single-channel images
        cv::split( image, image_channels );

        // get named references to each resulting image
        cv::Mat const & l_image = image_channels[0];
        cv::Mat const & a_image = image_channels[1];
        cv::Mat const & b_image = image_channels[2];

        // allocate an image with the height of the original and the width of 3 times the original
        cv::Mat combined_image( image.rows, 3 * image.cols, CV_8UC1 );
        // allocate an image for sub-matrices of the original image
        cv::Mat combined_image_sub;

        // get a reference to the first MxN sub-matrix in combined_image
        combined_image_sub = combined_image( cv::Rect( image.cols * 0, 0, image.cols, image.rows ) );
        // copy l_image to this sub-matrix
        l_image.copyTo( combined_image_sub );

        // get a reference to the second MxN sub-matrix in combined_image
        combined_image_sub = combined_image( cv::Rect( image.cols * 1, 0, image.cols, image.rows ) );
        // copy a_image to this sub-matrix
        a_image.copyTo( combined_image_sub );

        // get a reference to the third MxN sub-matrix in combined_image
        combined_image_sub = combined_image( cv::Rect( image.cols * 2, 0, image.cols, image.rows ) );
        // copy b_image to this sub-matrix
        b_image.copyTo( combined_image_sub );

        publishImages( "output_image", ImageProcPolicy::fromMat( combined_image, "", "mono8" ) );

        cv::imshow( "L_a_b", combined_image );

        cvWaitKey( 20 );
    }

    QUICKDEV_SPIN_ONCE()
    {

    }
};

#endif // NEUROMORPHICIMAGEPROC_LABIMAGEVIEW_H_
