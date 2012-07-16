/***************************************************************************
 *  include/seabee3_common/seabee_recognition_policy.h
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

#ifndef SEABEE3COMMON_SEABEERECOGNITIONPOLICY_H_
#define SEABEE3COMMON_SEABEERECOGNITIONPOLICY_H_

// policies
#include <quickdev/tf_tranceiver_policy.h>
#include <quickdev/node_handle_policy.h>

// objects
#include <seabee3_common/recognition_primitives.h>
#include <quickdev/multi_subscriber.h>

using namespace seabee;

QUICKDEV_DECLARE_POLICY_NS( SeabeeRecognition )
{
    typedef quickdev::TfTranceiverPolicy _TfTranceiverPolicy;
    typedef quickdev::NodeHandlePolicy _NodeHandlePolicy;
}

QUICKDEV_DECLARE_POLICY( SeabeeRecognition, _TfTranceiverPolicy, _NodeHandlePolicy )

QUICKDEV_DECLARE_POLICY_CLASS( SeabeeRecognition )
{
public:
    typedef QUICKDEV_GET_POLICY_NS( SeabeeRecognition )::_TfTranceiverPolicy _TfTranceiverPolicy;
    typedef QUICKDEV_GET_POLICY_NS( SeabeeRecognition )::_NodeHandlePolicy _NodeHandlePolicy;

    typedef quickdev::SimpleActionToken SimpleActionToken;

private:
    ros::MultiSubscriber<> multi_sub_;

protected:
    std::mutex find_landmark_mutex_;
    std::condition_variable find_landmark_condition_;

    _LandmarkArrayMsg::ConstPtr landmarks_msg_ptr_;
    std::mutex landmarks_mutex_;

    std::map<std::string, Landmark> landmarks_map_;
    std::mutex landmarks_map_mutex_;

    QUICKDEV_MAKE_POLICY_FUNCS( SeabeeRecognition )

    QUICKDEV_DECLARE_POLICY_CONSTRUCTOR( SeabeeRecognition )
    {
        //
    }

    QUICKDEV_ENABLE_INIT()
    {
        auto & nh_rel = _NodeHandlePolicy::getNodeHandle();

        multi_sub_.addSubscriber( nh_rel, "landmarks", &SeabeeRecognitionPolicy::landmarksCB, this );

        initPolicies<quickdev::policy::ALL>();

        QUICKDEV_SET_INITIALIZED();
    }

    QUICKDEV_DECLARE_MESSAGE_CALLBACK( landmarksCB, _LandmarkArrayMsg )
    {
        auto lock = quickdev::make_unique_lock( landmarks_mutex_, std::try_to_lock );

        if( !lock ) return;

        landmarks_msg_ptr_ = msg;

        find_landmark_condition_.notify_all();
    }

    SimpleActionToken findLandmark( Landmark const & target )
    {
        // save any conditions that may block the action, so we can unblock these conditions if the user cancels the action
        SimpleActionToken result( std::vector<std::condition_variable *>( { &find_landmark_condition_ } ) );
        result.start( quickdev::auto_bind( quickdev::auto_bind( &SeabeeRecognitionPolicy::findLandmarkImpl, this ), target, result ) );
        return result;
    }

    void findLandmarkImpl( Landmark const & target, SimpleActionToken token )
    {
        while( token.ok() && ros::ok() )
        {
            if( !landmarks_msg_ptr_ )
            {
                auto find_landmark_lock = quickdev::make_unique_lock( find_landmark_mutex_ );
                find_landmark_condition_.wait( find_landmark_lock );
            }

            if( !( token.ok() && ros::ok() ) )
            {
                token.cancel();
                return;
            }

            _LandmarkArrayMsg landmark_array_msg;
            {
                auto landmarks_lock = quickdev::make_unique_lock( landmarks_mutex_ );
                landmark_array_msg = *landmarks_msg_ptr_;
            }

            auto const & landmarks = landmark_array_msg.landmarks;

            auto landmarks_map_lock = quickdev::make_unique_lock( landmarks_map_mutex_);
            for( auto landmarks_it = landmarks.cbegin(); landmarks_it != landmarks.cend(); ++landmarks_it )
            {
                auto const & landmark_msg = *landmarks_it;

                Landmark landmark( landmark_msg );

                if( landmark == target )
                {
                    landmarks_map_[landmark_msg.name] = landmark;
                    token.complete( true );
                    return;
                }
            }
        }
    }

    // #########################################################################################################################################
    //! Update the existing landmark filter in-place
    /*!
     * - Passing a narrowing filter item follwed by a widening filter item will remove the narrowing filter item
     * - Passing a widening filter item followed by a narrowing filter item will remove the widening filter item
     * - Passing two or more narrowing filter items will join the filter items
     * - Passing an inverted filter item will widen the search, ie { Buoy(), -Buoy( RED ) } will result in all buoys except for red ones
     * - Passing only an inverted filter item will function as if Landmark() was prepended to the list of arguments
     *
     * - Passing just a Landmark() will clear the filter and accept any kind of landmark
     * - Passing a Buoy() will accept any kind of buoy
     * - Passing a Buoy( seabee_common::colors::RED ) will only accept red buoys
     *
     * - Passing { Landmark(), Buoy(), Pipe(), Buoy( RED ) } will result in only pipes and red buoys
     * - Passing { Landmark(), -Buoy( RED ), -Buoy( GREEN ), Pipe() } will result in pipes and yellow buoys
     */
    template<class... __Args>
    void updateLandmarkFilter( Landmark const & landmark, __Args&&... landmarks );

    // #########################################################################################################################################
    //! Only look for landmarks that match the given filter
    /*!
     * - Calling this function will reset the filter, then apply the given changes
     *
     * - To accept all landmarks, pass: Landmark()
     * - To accept no landmarks, pass: -Landmark()
     */
    template<class... __Args>
    void setLandmarkFilter( Landmark const & landmark, __Args&&... landmarks );

    // #########################################################################################################################################
    //! Set (not update) the landmark filter to the specified value and return the resulting landmarks
    template<class... __Args>
    std::map<std::string, Landmark> getLandmarks( Landmark const & landmark, __Args&&... landmarks );

    //! Just get the resulting landmarks
    std::map<std::string, Landmark> getLandmarks();

    /* This is the client's main means of accessing and updating a policy; un-comment and change args as appropriate

    void update( args... )
    {
        // When using init(), it's useful to be able to detect whether our init() function as been called
        // One option is to simply read the member "initialized_"
        // Alternatively, there are some simple macros for notifying/responding to situations where init() has not been called
        // These macros can be used anywhere within our policy as long as our init() function has been declared with QUICKDEV_ENABLE_INIT{}
        //
        // If we're not initialized yet, warn and continue
        // QUICKDEV_CHECK_INITIALIZED();
        //
        // If we're not initialized yet, warn and return immediately
        // QUICKDEV_ASSERT_INITIALIZED();
        //
    }*/
};

#endif // SEABEE3COMMON_SEABEERECOGNITIONPOLICY_H_
