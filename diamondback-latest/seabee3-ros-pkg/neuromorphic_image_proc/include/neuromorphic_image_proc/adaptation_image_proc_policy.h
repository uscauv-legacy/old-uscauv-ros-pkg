/***************************************************************************
 *  include/neuromorphic_image_proc/adaptation_image_proc_policy.h
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

#ifndef NEUROMORPHICIMAGEPROC_ADAPTATIONIMAGEPROCPOLICY_H_
#define NEUROMORPHICIMAGEPROC_ADAPTATIONIMAGEPROCPOLICY_H_

#include <quickdev/image_proc_policy.h>

QUICKDEV_DECLARE_POLICY_NS( AdaptationImageProc )
{
    typedef quickdev::ImageProcPolicy _ImageProcPolicy;
}

QUICKDEV_DECLARE_POLICY( AdaptationImageProc, _ImageProcPolicy )

QUICKDEV_DECLARE_POLICY_CLASS( AdaptationImageProc )
{
    // create utility functions for this policy
    //
    QUICKDEV_MAKE_POLICY_FUNCS( AdaptationImageProc )

    cv_bridge::CvImageConstPtr adaptation_mask_ptr_;

    QUICKDEV_DECLARE_POLICY_CONSTRUCTOR( AdaptationImageProc )
    {
        printPolicyActionStart( "create", this );

        preInit();

        printPolicyActionDone( "create", this );
    }

    void preInit()
    {
        QUICKDEV_GET_NODEHANDLE( nh_rel );

        // subscription to mask with pixel updates
        image_subs_.addSubscriber( nh_rel, "adaptation_mask", &AdaptationImageProcPolicy::adaptationImageCB, this, subscriber_storage_ );

        // publisher for modified mask
        image_pubs_.addPublishers<sensor_msgs::Image>( nh_rel, {"output_adaptation_mask"}, publisher_storage_ );
    }

    void adaptationImageCB( const sensor_msgs::Image::ConstPtr & image_msg )
    {
        try
        {
            adaptation_mask_ptr_ = cv_bridge::toCvShare( image_msg );
        }
        catch (cv_bridge::Exception& e)
        {
            PRINT_ERROR( "cv_bridge exception: %s", e.what() );
            return;
        }
    }
};

#endif // NEUROMORPHICIMAGEPROC_ADAPTATIONIMAGEPROCPOLICY_H_
