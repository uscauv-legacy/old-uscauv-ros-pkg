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

// declare a node called ColorClassifierTrainerNode
// a quickdev::RunablePolicy is automatically prepended to the list of policies our node will use
// to use more policies, simply list them here:
//
// QUICKDEV_DECLARE_NODE( ColorClassifierTrainer, SomePolicy1, SomePolicy2 )
//
QUICKDEV_DECLARE_NODE( ColorClassifierTrainer )

// declare a class called ColorClassifierTrainerNode
//
QUICKDEV_DECLARE_NODE_CLASS( ColorClassifierTrainer )
{
    // variable initializations can be appended to this constructor as a comma-separated list:
    //
    // QUICKDEV_DECLARE_NODE_CONSTRUCTOR( ColorClassifierTrainer ), member1_( some_value ), member2_( some_other_value ){}
    //
    QUICKDEV_DECLARE_NODE_CONSTRUCTOR( ColorClassifierTrainer )
    {
        //
    }

    // this function is called by quickdev::RunablePolicy after all policies are constructed but just before the main loop is started
    // all policy initialization should be done here
    //
    QUICKDEV_SPIN_FIRST()
    {
        // say we had a policy called _SomePolicy that looked for the meta-parameter "some_value1_param" of type SomeType and
        // "some_value2_param" of type SomeOtherType in its init function
        // we can create those meta-params here and then pass them to all policies using initAll():
        //
        // initAll( "some_value1_param", SomeType(), "some_value2_param", SomeOtherType() );
        //
        // or we can pass those meta-params only to _SomePolicy using its init() function:
        //
        // _SomePolicy::init( "some_value1_param", SomeType(), "some_value2_param", SomeOtherType() );
        //
        // if we don't want to initialize all policies and use their default values, we can simply call initAll() with no arguments
        // note that most initable policies won't function properly unless their init() functions are called
        // therefore, to get the default behavior from all policies, be sure to call initAll()
        //
        initAll();
    }

    // this opitonal function is called by quickdev::RunablePolicy at a fixed rate (defined by the ROS param _loop_rate)
    // most updateable policies should have their update( ... ) functions called within this context
    //
    QUICKDEV_SPIN_ONCE()
    {
        //
    }
};

#endif // COLORCLASSIFIER_COLORCLASSIFIERTRAINERNODE_H_
