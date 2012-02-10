/***************************************************************************
 *  include/seabee3_physics/seabee3_physics_node.h
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

#ifndef SEABEE3PHYSICS_SEABEE3PHYSICSNODE_H_
#define SEABEE3PHYSICS_SEABEE3PHYSICSNODE_H_

#include <quickdev/node.h>

#include <quickdev/reconfigure_policy.h>
#include <quickdev/tf_tranceiver_policy.h>

#include <quickdev/time.h>
#include <quickdev/multi_publisher.h>
#include <quickdev/multi_subscriber.h>

#include <seabee3_common/movement.h>

#include "btBulletDynamicsCommon.h"

#include <seabee3_driver/MotorVals.h>
#include <seabee3_driver/KillSwitch.h>

#include <seabee3_physics/Seabee3PhysicsConfig.h>

using namespace seabee3_common;

typedef seabee3_driver::MotorVals _MotorValsMsg;
typedef seabee3_driver::KillSwitch _KillSwitchMsg;
typedef seabee3_physics::Seabee3PhysicsConfig _Seabee3PhysicsConfig;

typedef quickdev::ReconfigurePolicy<_Seabee3PhysicsConfig> _Seabee3PhysicsLiveParams;
typedef quickdev::TfTranceiverPolicy _TfTranceiverPolicy;

QUICKDEV_DECLARE_NODE( Seabee3Physics, _Seabee3PhysicsLiveParams, _TfTranceiverPolicy )

QUICKDEV_DECLARE_NODE_CLASS( Seabee3Physics )
{
    ros::MultiPublisher<> multi_pub_;
    ros::MultiSubscriber<> multi_sub_;

    boost::shared_ptr<btBroadphaseInterface> broadphase_;
    boost::shared_ptr<btDefaultCollisionConfiguration> collision_configuration_;
    boost::shared_ptr<btCollisionDispatcher> dispatcher_;
    boost::shared_ptr<btSequentialImpulseConstraintSolver> solver_;
    boost::shared_ptr<btDiscreteDynamicsWorld> dynamics_world_;
    boost::shared_ptr<btCollisionShape> seabee_shape_;
    boost::shared_ptr<btDefaultMotionState> seabee_motion_state_;
    boost::shared_ptr<btRigidBody> seabee_body_;

    quickdev::Timer timer_;

    std::vector<btTransform> thruster_transforms_;
    std::vector<int> thruster_values_;
    bool is_killed_;

    QUICKDEV_DECLARE_NODE_CONSTRUCTOR( Seabee3Physics ),
        thruster_transforms_( movement::NUM_THRUSTERS ),
        thruster_values_( movement::NUM_THRUSTERS ),
        is_killed_( true )
    {
        // Build the broadphase
        broadphase_ = quickdev::make_shared( new btDbvtBroadphase() );

        // Set up the collision configuration and dispatcher
        collision_configuration_ = quickdev::make_shared( new btDefaultCollisionConfiguration() );
        dispatcher_ = quickdev::make_shared( new btCollisionDispatcher( collision_configuration_.get() ) );

        // The actual physics solver
        solver_ = quickdev::make_shared( new btSequentialImpulseConstraintSolver );

        // The world
        dynamics_world_ = quickdev::make_shared( new btDiscreteDynamicsWorld( dispatcher_.get(), broadphase_.get(), solver_.get(), collision_configuration_.get() ) );
        dynamics_world_->setGravity( btVector3( 0, 0, 0 ) );

        // Seabee's Body
        seabee_shape_ = quickdev::make_shared( new btCylinderShape( btVector3( .5, .2, .2 ) ) );

        seabee_motion_state_ = quickdev::make_shared( new btDefaultMotionState( btTransform( btQuaternion( 0, 0, 0, 1 ), btVector3( 0, 0, 0 ) ) ) );

        // She weighs 35kg
        btScalar seabee_mass = 100;
        btVector3 seabee_inertia( 0, 0, 0 );
        seabee_shape_->calculateLocalInertia( seabee_mass, seabee_inertia );

        btRigidBody::btRigidBodyConstructionInfo seabee_body_ci( seabee_mass, seabee_motion_state_.get(), seabee_shape_.get(), seabee_inertia );
        seabee_body_ = quickdev::make_shared( new btRigidBody( seabee_body_ci ) );
        seabee_body_->setActivationState( DISABLE_DEACTIVATION );
        seabee_body_->setDamping( 0.018, 0 );

        dynamics_world_->addRigidBody( seabee_body_.get() );
    }

    QUICKDEV_SPIN_FIRST()
    {
        QUICKDEV_GET_RUNABLE_NODEHANDLE( nh_rel );

        multi_sub_.addSubscriber( nh_rel, "motor_vals", &Seabee3PhysicsNode::motorValsCB, this );
        multi_sub_.addSubscriber( nh_rel, "kill_switch", &Seabee3PhysicsNode::killSwitchCB, this );

        fetchThrusterTransforms();

        timer_.reset();

        initPolicies<quickdev::policy::ALL>();
    }

    void fetchThrusterTransforms()
    {
        // Fill in all of our thruster transforms
        for( size_t i = 0; i < movement::NUM_THRUSTERS; ++i )
        {
            if( !movement::MotorControllerIDs::isThruster( i ) ) continue;

            std::ostringstream stream;
            stream << "/seabee3/thruster" << i;

            thruster_transforms_[i] = lookupTransform( "/seabee3/base_link", stream.str() );
        }
    }

    QUICKDEV_SPIN_ONCE()
    {
        auto const & thrust_to_force = config_.thrust_to_force;
        auto const & is_killed = is_killed_;

        btTransform world_transform;

        seabee_body_->clearForces();

        seabee_body_->getMotionState()->getWorldTransform( world_transform );

        for ( size_t i = 0; i < movement::NUM_THRUSTERS; i++ )
        {
            if( !movement::MotorControllerIDs::isThruster( i ) ) continue;

            auto const thrust = is_killed ? 0.0 : thruster_values_.at(i) * thrust_to_force;

            auto const & current_thruster_tf = thruster_transforms_.at(i);

            // our thrust comes out of the x-axis of the motor
            btVector3 const force_vec( thrust, 0, 0 );

            // the position of the thruster with respect to the center of the sub
            btVector3 const & force_rel_pos = current_thruster_tf.getOrigin();

            ROS_INFO( "MOTOR%Zu THRUST: %f (%f,%f,%f) @ (%f,%f,%f)",
                      i,
                      thrust,
                      force_vec.x(),
                      force_vec.y(),
                      force_vec.z(),
                      force_rel_pos.x(),
                      force_rel_pos.y(),
                      force_rel_pos.z() );

            // the direction of the thruster with respect to the world
            btTransform const force_rotation( world_transform.getRotation() * current_thruster_tf.getRotation() );

            // the final, rotated force
            auto const rotated_force( force_rotation * force_vec );

            // apply @rotated_force at @force_rel_pos
            seabee_body_->applyForce( rotated_force, force_rel_pos );
        }

        btVector3 const & lin_vel = seabee_body_->getLinearVelocity();
        //btVector3 force_drag = -lin_v_ * reconfigure_params_.drag_constant;
        btVector3 force_drag;

        force_drag.setX( 0.5 * 1000 * 8.636 * lin_vel.x() * lin_vel.x() * 0.81 );
        force_drag.setY( 0.5 * 1000 * 1.143 * lin_vel.y() * lin_vel.y() * 0.42 );
        force_drag.setZ( 0.5 * 1000 * 1.906 * lin_vel.z() * lin_vel.z() * 0.42 );

        seabee_body_->applyForce( force_drag, seabee_body_->getCenterOfMassPosition() );

        ROS_INFO( "lin_vel: %f, %f, %f", lin_vel.x(), lin_vel.y(), lin_vel.z() );
        ROS_INFO( "Drag Force: %f ... Sub Speed: %f", force_drag.length(), lin_vel.length() );

        auto const & dt = timer_.update();

        //seabee_body_->applyDamping(dt);

        // Step the physics simulation; increase the simulation's time by dt (dt is variable); interpolate by up to 2 steps when dt is not the ideal @loop_rate_seconds_ time
        dynamics_world_->stepSimulation( dt, 2, getLoopRateSeconds() );

        //tf::Transform givens_tf;
        //tf_utils::fetchTfFrame( givens_tf, "/landmark_map", "/seabee3/base_link_givens" );
//      geometry_msgs::Twist givens;
//      givens_tf >> givens;

//      btTransform center_of_mass;
//
//      center_of_mass = seabee_body_->getCenterOfMassTransform();
//      center_of_mass.setRotation( givens_tf.getRotation() );
//      center_of_mass.getOrigin().setZ( givens_tf.getOrigin().getZ() );
//      seabee_body_->setCenterOfMassTransform( center_of_mass );

/*        seabee_body_->getMotionState()->getWorldTransform( world_transform );
        world_transform.setRotation( givens_tf.getRotation() );
        world_transform.getOrigin().setZ( givens_tf.getOrigin().getZ() );
        seabee_body_->getMotionState()->setWorldTransform( world_transform );



        seabee_body_->setAngularVelocity( btVector3() );
        btVector3 linVel = seabee_body_->getLinearVelocity();
    linVel[2] = 0;
    seabee_body_->setLinearVelocity(linVel);*/

        seabee_body_->getMotionState()->getWorldTransform( world_transform );
        //code to factor in drag

        printf( "Body Pos: %f %f %f\n",
                world_transform.getOrigin().getX(),
                world_transform.getOrigin().getY(),
                world_transform.getOrigin().getZ() );

        publishTransform( world_transform, "/world", "/seabee3/physics_link" );
    }

    QUICKDEV_DECLARE_MESSAGE_CALLBACK( motorValsCB, _MotorValsMsg )
    {
        for( size_t i = 0; i < movement::NUM_THRUSTERS; ++i )
        {
            if( msg->mask.at(i) ) thruster_values_[i] = msg->motors.at(i);
        }
    }

    QUICKDEV_DECLARE_MESSAGE_CALLBACK( killSwitchCB, _KillSwitchMsg )
    {
        is_killed_ = msg->is_killed;
    }
};

#endif // SEABEE3PHYSICS_SEABEE3PHYSICSNODE_H_
