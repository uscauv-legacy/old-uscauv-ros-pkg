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

// objects
#include <mutex>

// policies
#include <quickdev/callback_policy.h>

QUICKDEV_DECLARE_POLICY_NS( AdaptationImageProc )
{
    typedef quickdev::ImageProcPolicy _ImageProcPolicy;
    typedef quickdev::CallbackPolicy<void( cv_bridge::CvImageConstPtr const &, cv_bridge::CvImageConstPtr const & )> _CombinedImageCallbackPolicy;
}

QUICKDEV_DECLARE_POLICY( AdaptationImageProc, _ImageProcPolicy, _CombinedImageCallbackPolicy )

QUICKDEV_DECLARE_POLICY_CLASS( AdaptationImageProc )
{
    // create utility functions for this policy
    //
    QUICKDEV_MAKE_POLICY_FUNCS( AdaptationImageProc )

    typedef QUICKDEV_GET_POLICY_NS( AdaptationImageProc )::_ImageProcPolicy _ImageProcPolicy;
    typedef QUICKDEV_GET_POLICY_NS( AdaptationImageProc )::_CombinedImageCallbackPolicy _CombinedImageCallbackPolicy;
    typedef cv_bridge::CvImageConstPtr _CvImageMsgPtr;

    _CvImageMsgPtr image_ptr_;
    _CvImageMsgPtr adaptation_mask_ptr_;

    std::deque<_CvImageMsgPtr> image_queue_;
    std::deque<_CvImageMsgPtr> mask_queue_;

    std::deque<_CvImageMsgPtr> visited_images_;
    std::deque<_CvImageMsgPtr> visited_masks_;

    std::mutex synced_callback_mutex_;

    QUICKDEV_DECLARE_POLICY_CONSTRUCTOR( AdaptationImageProc ),
        initialized_( false )
    {
        printPolicyActionStart( "create", this );
        printPolicyActionDone( "create", this );
    }

    QUICKDEV_ENABLE_INIT()
    {
        initPolicies<_ImageProcPolicy>
        (
            "image_callback_param", quickdev::auto_bind( &AdaptationImageProcPolicy::imageCB, this )
        );

        // subscription to mask with pixel updates
        _ImageProcPolicy::addImageSubscriber( "adaptation_mask", quickdev::auto_bind( &AdaptationImageProcPolicy::adaptationImageCB, this ) );

        // publisher for modified mask
        _ImageProcPolicy::addImagePublisher( "output_adaptation_mask" );

        QUICKDEV_SET_INITIALIZED();
    }

    template<class... __Args>
    void registerCombinedImageCallback( __Args&&... args )
    {
        _CombinedImageCallbackPolicy::registerCallback( std::forward<__Args>( args )... );
    }

    void tryCallSyncedImagesCB()
    {
//        std::cout << "Looking for image/mask pair; waiting for lock..." << std::endl;
        synced_callback_mutex_.lock();
//        std::cout << "Looking for image/mask pair; acquired lock..." << std::endl;

//        std::cout << "Trying to find mask/image pair; cached " << visited_images_.size() << " old images and " << visited_masks_.size() << " old masks" << std::endl;

/*
        in    vis

        i m | i m
        ----|----
        0 2 | - -
        1 3 | - -
        2 4 | - -
        3 0 | - -
        4 1 | - -

        i m | i m
        ----|----
        1 3 | 0 2
        2 4 | - -
        3 0 | - -
        4 1 | - -

        i m | i m
        ----|----
        2 4 | 0 2
        3 0 | 1 3
        4 1 | - -

        i m | i m
        ----|----
        3 4 | 0 3
        4 0 | 1 -
        - 1 | - -

        i m | i m
        ----|----
        4 4 | 0 -
        - 0 | 1 -
        - 1 | - -

        i m | i m
        ----|----
        - 0 | 0 -
        - 1 | 1 -

        i m | i m
        ----|----
        - 0 | 0 -

        i m | i m
        ----|----
*/
        //! storage for a copy of the current image
        _CvImageMsgPtr current_image;
        //! storage for a copy of the current mask
        _CvImageMsgPtr current_mask;

        //! storage for a copy of the image, if we find it
        _CvImageMsgPtr matched_image;
        //! storage for a copy of the mask, if we find it
        _CvImageMsgPtr matched_mask;
        //! flag to indicate whether we've found the pair of images
        bool match_found = false;

        // while no match has been found and there's at least one new image or mask to process
        while( !match_found && !( image_queue_.empty() && mask_queue_.empty() ) )
        {
            // if we have a new image
            if( !image_queue_.empty() )
            {
//                std::cout << "Looking for image/mask pair; using new image" << std::endl;
                current_image = image_queue_.front();
                image_queue_.pop_front();
            }

            // if we have a new mask
            if( !mask_queue_.empty() )
            {
//                std::cout << "Looking for image/mask pair; using new mask" << std::endl;
                current_mask = mask_queue_.front();
                mask_queue_.pop_front();
            }

            // if the first pair of images matches
            if( current_image && current_mask && current_image->header.stamp == current_mask->header.stamp )
            {
//                std::cout << "Looking for image/mask pair; new messages match" << std::endl;
                match_found = true;
                matched_image = current_image;
                matched_mask = current_mask;
            }
            else
            {
//                std::cout << "Looking for image/mask pair; new messages don't match; looking through " << visited_images_.size() << " old images and " << visited_masks_.size() << " old masks" << std::endl;
                // if we have a new mask
                if( !match_found && current_mask )
                {
                    // try to find a matching image
                    for( auto visited_images_it = visited_images_.begin(); visited_images_it != visited_images_.end(); ++visited_images_it )
                    {
                        if( current_mask->header.stamp == (*visited_images_it)->header.stamp )
                        {
//                            std::cout << "Looking for image/mask pair; matched current mask to old image" << std::endl;
                            match_found = true;
                            matched_image = *visited_images_it;
                            matched_mask = current_mask;

                            visited_images_.erase( visited_images_it );

                            break;
                        }
                    }
                }
                // if we have a new image
                if( !match_found && current_image )
                {
                    // try to find a matching mask
                    for( auto visited_masks_it = visited_masks_.begin(); visited_masks_it != visited_masks_.end(); ++visited_masks_it )
                    {
                        if( current_image->header.stamp == (*visited_masks_it)->header.stamp )
                        {
//                            std::cout << "Looking for image/mask pair; matched current image to old mask" << std::endl;
                            match_found = true;
                            matched_image = current_image;
                            matched_mask = *visited_masks_it;

                            visited_masks_.erase( visited_masks_it );

                            break;
                        }
                    }
                }

                // if we didn't find any matches, or if we did but the current image wasn't matched
                if( current_image && ( !match_found || current_image != matched_image ) ) visited_images_.push_back( current_image );
                // if we didn't find any matches, or if we did but the current mask wasn't matched
                if( current_mask && ( !match_found || current_mask != matched_mask ) ) visited_masks_.push_back( current_mask );
            }
        }

        // if we found a match, pass it to the registered caller
        if( match_found )
        {
//            std::cout << "Looking for image/mask pair; found matching pair; invoking registered callback" << std::endl;
            _CombinedImageCallbackPolicy::invokeCallback( matched_image, matched_mask );
//            std::cout << "Looking for image/mask pair; found matching pair; registered callback completed" << std::endl;
        }

        // unblock dependent callbacks
//        std::cout << "Looking for image/mask pair; releasing lock..." << std::endl;
        synced_callback_mutex_.unlock();
    }

    QUICKDEV_DECLARE_IMAGE_CALLBACK( imageCB )
    {
//        std::cout << "Got new image; waiting for lock..." << std::endl;
        synced_callback_mutex_.lock();
//        std::cout << "Got new image; acquired lock" << std::endl;

        //while( image_queue_.size() >= 10 ) image_queue_.pop_front();
        image_queue_.push_back( image_msg );

//        PRINT_INFO( "%zu images waiting to be processed.", image_queue_.size() );

//        std::cout << "Got new image; releasing lock" << std::endl;
        synced_callback_mutex_.unlock();

        tryCallSyncedImagesCB();
    }

    QUICKDEV_DECLARE_IMAGE_CALLBACK( adaptationImageCB )
    {
//        std::cout << "Got new mask; waiting for lock..." << std::endl;
        synced_callback_mutex_.lock();
//        std::cout << "Got new mask; acquired lock" << std::endl;

        //while( mask_queue_.size() >= 10 ) adaptation_mask_queue_.pop_front();
        mask_queue_.push_back( image_msg );

//        PRINT_INFO( "%zu masks waiting to be processed.", mask_queue_.size() );

//        std::cout << "Got new mask; releasing lock" << std::endl;
        synced_callback_mutex_.unlock();

        tryCallSyncedImagesCB();
    }
};

#endif // NEUROMORPHICIMAGEPROC_ADAPTATIONIMAGEPROCPOLICY_H_
