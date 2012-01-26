/***************************************************************************
 *  include/seabee3_common/ImageArrayViewer_node.h
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

#ifndef SEABEE3COMMON_IMAGEARRAYVIEWERNODE_H_
#define SEABEE3COMMON_IMAGEARRAYVIEWERNODE_H_

#include <quickdev/node.h>
#include <quickdev/image_proc_policy.h>
#include <seabee3_common/NamedImageArray.h>

QUICKDEV_DECLARE_NODE( ImageArrayViewer )

QUICKDEV_DECLARE_NODE_CLASS( ImageArrayViewer )
{
    typedef seabee3_common::NamedImageArray _NamedImageArrayMsg;
    ros::MultiSubscriber<> multi_sub_;

    QUICKDEV_DECLARE_NODE_CONSTRUCTOR( ImageArrayViewer )
    {
        //
    }

    QUICKDEV_SPIN_FIRST()
    {
        initAll();

        QUICKDEV_GET_RUNABLE_NODEHANDLE( nh_rel );

        multi_sub_.addSubscriber( nh_rel, "images", &ImageArrayViewerNode::imagesCB, this );
    }

    QUICKDEV_DECLARE_MESSAGE_CALLBACK( imagesCB, _NamedImageArrayMsg )
    {
        for( auto named_image_msg = msg->images.begin(); named_image_msg != msg->images.end(); ++named_image_msg )
        {
            cv::namedWindow( named_image_msg->name, 0 );

            auto cv_image_ptr = quickdev::opencv_conversion::fromImageMsg( named_image_msg->image );
            cv::Mat const & image = cv_image_ptr->image;

            cv::imshow( named_image_msg->name, image );
        }

        cvWaitKey( 20 );
    }

    QUICKDEV_SPIN_ONCE()
    {
        //
    }
};

#endif // SEABEE3COMMON_IMAGEARRAYVIEWERNODE_H_
