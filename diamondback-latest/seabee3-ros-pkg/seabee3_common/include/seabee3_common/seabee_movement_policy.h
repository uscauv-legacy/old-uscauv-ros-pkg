/***************************************************************************
 *  include/seabee3_common/seabee_movement_policy.h
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

#ifndef SEABEE3COMMON_SEABEEMOVEMENTPOLICY_H_
#define SEABEE3COMMON_SEABEEMOVEMENTPOLICY_H_

// policies
#include <quickdev/tf_tranceiver_policy.h>

// objects
#include <seabee3_common/motion_primitives.h>

// actions
//#include <seabee3_common/

using namespace seabee;

QUICKDEV_DECLARE_POLICY_NS( SeabeeMovement )
{
    typedef quickdev::TfTranceiverPolicy _TfTranceiverPolicy;
}

QUICKDEV_DECLARE_POLICY( SeabeeMovement, _TfTranceiverPolicy )

QUICKDEV_DECLARE_POLICY_CLASS( SeabeeMovement )
{
    QUICKDEV_MAKE_POLICY_FUNCS( SeabeeMovement )

    QUICKDEV_DECLARE_POLICY_CONSTRUCTOR( SeabeeMovement )
    {
        //
    }

    QUICKDEV_ENABLE_INIT()
    {
        initPolicies<quickdev::policy::ALL>();

        QUICKDEV_SET_INITIALIZED();
    }

    // #########################################################################################################################################
    //! Return the Pose of the landmark with the given name
    Pose getPose( std::string const & frame_name );

    //! Return the Pose of the sub
    /*!
     * - Calls getPose( "seabee" )
     */
    Pose getCurrentPose();

    // #########################################################################################################################################
    //! Fire the given device
    quickdev::ActionToken<boost::thread> fireDevice( FiringDevice const & device );

    // #########################################################################################################################################
    //! Fire the given shooter
    /*!
     * - Calls fireDevice( Shooter( shooter_id ) )
     */
    quickdev::ActionToken<boost::thread> fireShooter( Shooter::Id const & shooter_id );

    //! Fire the given dropper
    /*!
     * - Calls fireDevice( Dropper( dropper_id ) )
     */
    quickdev::ActionToken<boost::thread> fireDropper( Dropper::Id const & dropper_id );

    // #########################################################################################################################################
    //! Move to the given pose
    /*!
     * - moveTo( getCurrentPose() + Orientation( 90 ) ) will rotate 90 degrees CCW (see faceTo( Orientation ) )
     * - moveTo( getCurrentPose() + Position( 0, 0, -1 ) will dive one meter
     */
    quickdev::ActionToken<boost::thread> moveTo( Pose const & pose );

    //! Move to some relative position
    /*!
     * - Same as moveTo( getCurrentPose() + position )
     */
    quickdev::ActionToken<boost::thread> moveTo( Position const & position );

    //! Move to some relative orientation
    /*!
     * - Same as moveTo( getCurrentPose() + orientation )
     */
    quickdev::ActionToken<boost::thread> moveTo( Orientation const & orientation );

    // #########################################################################################################################################
    //! Face the given position
    quickdev::ActionToken<boost::thread> faceTo( Position const & position );

    //! Face at the given orientation
    quickdev::ActionToken<boost::thread> faceTo( Orientation const & orientation );

    // #########################################################################################################################################
    //! Strafe around pose.position at current distance until our orientation matches pose.orientation + Degrees( 180 )
    quickdev::ActionToken<boost::thread> strafeAround( Pose const & pose );

    //! Strafe around pose.position at current distance until our orientation matches orientation
    quickdev::ActionToken<boost::thread> strafeAround( Pose const & pose, Orientation const & orientation );

    //! Strafe around pose.position at distance until ... (see above)
    quickdev::ActionToken<boost::thread> strafeAround( Pose const & pose, double distance, ... );

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

#endif // SEABEE3COMMON_SEABEEMOVEMENTPOLICY_H_
