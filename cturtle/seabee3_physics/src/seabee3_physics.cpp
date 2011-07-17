/*******************************************************************************
 *
 *      seabee3_physics
 * 
 *      Copyright (c) 2010
 *
 *      Edward T. Kaszubski (ekaszubski@gmail.com)
 *      Randolph C. Voorhies (voorhies at usc dot edu)
 *
 *      All rights reserved.
 *
 *      Redistribution and use in source and binary forms, with or without
 *      modification, are permitted provided that the following conditions are
 *      met:
 *      
 *      * Redistributions of source code must retain the above copyright
 *        notice, this list of conditions and the following disclaimer.
 *      * Redistributions in binary form must reproduce the above
 *        copyright notice, this list of conditions and the following disclaimer
 *        in the documentation and/or other materials provided with the
 *        distribution.
 *      * Neither the name of the USC Underwater Robotics Team nor the names of its
 *        contributors may be used to endorse or promote products derived from
 *        this software without specific prior written permission.
 *      
 *      THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *      "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *      LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *      A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *      OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *      SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *      LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *      DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *      THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *      (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *      OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *******************************************************************************/

#include <ros/ros.h>
#include <tf/tf.h>
#include "btBulletDynamicsCommon.h"
#include <base_tf_tranceiver/base_tf_tranceiver.h>
#include <vector>
#include <sstream>
#include <seabee3_common/movement_common.h>
#include <seabee3_driver_base/MotorCntl.h>
#include <seabee3_common/PhysicsState.h>
#include <seabee3_physics/Seabee3PhysicsConfig.h>
#include <seabee3_driver_base/KillSwitch.h>
#include <std_srvs/Empty.h>

typedef seabee3_physics::Seabee3PhysicsConfig _ReconfigureType;

class Seabee3Physics: public BaseTfTranceiver<_ReconfigureType>
{
public:
	const static int _NUM_MOTOR_CONTROLLERS = movement_common::NUM_MOTOR_CONTROLLERS;

private:
	std::vector<geometry_msgs::Twist> thruster_transforms_;
	std::string thruster_transform_name_prefix_;

	// Bullet related stuff
	btBroadphaseInterface *broadphase_;
	btDefaultCollisionConfiguration *collision_configuration_;
	btCollisionDispatcher *dispatcher_;
	btSequentialImpulseConstraintSolver *solver_;
	btDiscreteDynamicsWorld *dynamics_world_;
	btCollisionShape *seabee_shape_;
	btDefaultMotionState *seabee_motion_state_;
	btRigidBody *seabee_body_;

	ros::Subscriber kill_switch_sub_;
	ros::Subscriber motor_cntl_sub_;
	ros::Publisher physics_state_pub_;
  ros::ServiceServer reset_pose_svr_;
	std::vector<int> thruster_vals_;
	bool is_killed_;

	ros::Time last_call_;

public:
	Seabee3Physics( ros::NodeHandle &nh ) :
			BaseTfTranceiver<_ReconfigureType>( nh ), thruster_transforms_( _NUM_MOTOR_CONTROLLERS ), thruster_transform_name_prefix_( "/seabee3/thruster" ), thruster_vals_( _NUM_MOTOR_CONTROLLERS ), is_killed_( true )
	{

		reset_pose_svr_ = nh_local_.advertiseService( "reset_pose", &Seabee3Physics::resetPoseCB, this );

		ros::Duration( 0.5 ).sleep(); //wait for this frame to register on the network

		fetchThrusterTransforms();

		publishTfFrame( tf::Transform( tf::Quaternion( 0,
		                                               0,
		                                               0 ),
		                               tf::Vector3( 0,
		                                            0,
		                                            0 ) ),
		                "/landmark_map",
		                "/seabee3/physics_link" );

		motor_cntl_sub_ = nh.subscribe( "/seabee3/motor_cntl",
		                                1,
		                                &Seabee3Physics::motorCntlCB,
		                                this );
		physics_state_pub_ = nh.advertise<seabee3_common::PhysicsState>( std::string( "/seabee3/physics_state" ),
		                                                                 1 );

		kill_switch_sub_ = nh_local_.subscribe( "/seabee3/kill_switch", 2, &Seabee3Physics::killSwitchCB, this );

		// Build the broadphase
		broadphase_ = new btDbvtBroadphase();

		// Set up the collision configuration and dispatcher
		collision_configuration_ = new btDefaultCollisionConfiguration();
		dispatcher_ = new btCollisionDispatcher( collision_configuration_ );

		// The actual physics solver
		solver_ = new btSequentialImpulseConstraintSolver;

		// The world
		dynamics_world_ = new btDiscreteDynamicsWorld( dispatcher_,
		                                               broadphase_,
		                                               solver_,
		                                               collision_configuration_ );
		dynamics_world_->setGravity( btVector3( 0,
		                                        0,
		                                        0 ) );

		// Seabee's Body
		seabee_shape_ = new btCylinderShape( btVector3( .5,
		                                                .2,
		                                                .2 ) );

		seabee_motion_state_ = new btDefaultMotionState( btTransform( btQuaternion( 0,
		                                                                            0,
		                                                                            0,
		                                                                            1 ),
		                                                              btVector3( 0,
		                                                                         0,
		                                                                         0 ) ) );

		// She weighs 35kg
		btScalar seabee_mass = 100;
		btVector3 seabee_inertia( 0,
		                          0,
		                          0 );
		seabee_shape_->calculateLocalInertia( seabee_mass,
		                                      seabee_inertia );

		btRigidBody::btRigidBodyConstructionInfo seabee_body_ci( seabee_mass,
		                                                         seabee_motion_state_,
		                                                         seabee_shape_,
		                                                         seabee_inertia );
		seabee_body_ = new btRigidBody( seabee_body_ci );
		seabee_body_->setActivationState( DISABLE_DEACTIVATION );
		seabee_body_->setDamping( .018,
		                          0 );

		dynamics_world_->addRigidBody( seabee_body_ );

		last_call_ = ros::Time::now();
	}

	~Seabee3Physics()
	{
		delete dynamics_world_;
		delete solver_;
		delete dispatcher_;
		delete collision_configuration_;
		delete broadphase_;
		delete seabee_motion_state_;
		delete seabee_body_;
	}
	void killSwitchCB( const seabee3_driver_base::KillSwitch::ConstPtr & kill_switch_msg )
	{
		is_killed_ = kill_switch_msg->is_killed;
	}

	void spinOnce()
	{

		btTransform world_transform;

			seabee_body_->clearForces();

			seabee_body_->getMotionState()->getWorldTransform( world_transform );

			btTransform world_rotation;
			world_rotation.setRotation( world_transform.getRotation() );

			for ( size_t i = 0; i < _NUM_MOTOR_CONTROLLERS; i++ )
			{
				if ( i == movement_common::MotorControllerIDs::DROPPER_STAGE1 || i == movement_common::MotorControllerIDs::DROPPER_STAGE2
					    || i == movement_common::MotorControllerIDs::SHOOTER ) continue;

				float thrust = is_killed_ ? 0.0 : thruster_vals_[i] * reconfigure_params_.thrust_to_force;

				geometry_msgs::Vector3 pos = thruster_transforms_[i].linear;
				geometry_msgs::Vector3 ori = thruster_transforms_[i].angular;

				btVector3 force;
				force.setY( thrust );

				// Y P R
				btTransform force_tf( btQuaternion( ori.z,
					                                ori.y,
					                                ori.x ) );

				force_tf.setRotation( world_rotation * force_tf.getRotation() );

				btVector3 rel_pos;
				rel_pos.setX( pos.x );
				rel_pos.setY( pos.y );
				rel_pos.setZ( pos.z );

				ROS_INFO( "MOTOR%Zu THRUST: %f (%f,%f,%f) @ (%f,%f,%f)",
					      i,
					      thrust,
					      force.x(),
					      force.y(),
					      force.z(),
					      rel_pos.x(),
					      rel_pos.y(),
					      rel_pos.z() );

				seabee_body_->applyForce( force_tf * force,
					                      rel_pos );
			}

			btVector3 lin_v_ = seabee_body_->getLinearVelocity();
			btVector3 force_drag = -lin_v_ * reconfigure_params_.drag_constant;
			if ( force_drag[0] != force_drag[0] )
			{
				force_drag[0] = 0.0;
				force_drag[1] = 0.0;
				force_drag[2] = 0.0;
			}
			seabee_body_->applyForce( force_drag,
				                      seabee_body_->getCenterOfMassPosition() );
			ROS_INFO( "lin_v_       %f, %f, %f",
				      lin_v_[0],
				      lin_v_[1],
				      lin_v_[2] );
			ROS_INFO( "Drag Force: %f ... Sub Speed: %f",
				      force_drag.length(),
				      lin_v_.length() );

			double dt = ( ros::Time::now() - last_call_ ).toSec();

			//seabee_body_->applyDamping(dt);

			// Step the physics simulation
			dynamics_world_->stepSimulation( dt,
				                             50,
				                             1.0 / rate_ );
			last_call_ = ros::Time::now();

		tf::Transform givens_tf;
		tf_utils::fetchTfFrame( givens_tf,
		                        "/landmark_map",
		                        "/seabee3/base_link_givens" );
//		geometry_msgs::Twist givens;
//		givens_tf >> givens;

//		btTransform center_of_mass;
//
//		center_of_mass = seabee_body_->getCenterOfMassTransform();
//		center_of_mass.setRotation( givens_tf.getRotation() );
//		center_of_mass.getOrigin().setZ( givens_tf.getOrigin().getZ() );
//		seabee_body_->setCenterOfMassTransform( center_of_mass );

		seabee_body_->getMotionState()->getWorldTransform( world_transform );
		world_transform.setRotation( givens_tf.getRotation() );
		world_transform.getOrigin().setZ( givens_tf.getOrigin().getZ() );
		seabee_body_->getMotionState()->setWorldTransform( world_transform );



		seabee_body_->setAngularVelocity( btVector3() );
		btVector3 linVel = seabee_body_->getLinearVelocity();
    linVel[2] = 0;
    seabee_body_->setLinearVelocity(linVel);

		seabee_body_->getMotionState()->getWorldTransform( world_transform );
		//code to factor in drag

		printf( "Body Pos: %f %f %f\n",
		        world_transform.getOrigin().getX(),
		        world_transform.getOrigin().getY(),
		        world_transform.getOrigin().getZ() );

		publishTfFrame( world_transform,
		                "/landmark_map",
		                "/seabee3/physics_link" );
	}

	void fetchThrusterTransforms()
	{
		// Fill in all of our thruster transforms
		for ( size_t i = 0; i < _NUM_MOTOR_CONTROLLERS; ++i )
		{
			if ( i == movement_common::MotorControllerIDs::DROPPER_STAGE1 || i == movement_common::MotorControllerIDs::DROPPER_STAGE2
			        || i == movement_common::MotorControllerIDs::SHOOTER ) continue;
			std::ostringstream stream;
			stream << thruster_transform_name_prefix_ << i;
			tf::Transform tmp_transform;
			fetchTfFrame( tmp_transform,
			              "/seabee3/base_link",
			              stream.str() );
			tmp_transform >> thruster_transforms_[i];
		}
	}

	void motorCntlCB( const seabee3_driver_base::MotorCntlConstPtr &msg )
	{
		for ( size_t i = 0; i < _NUM_MOTOR_CONTROLLERS; i++ )
		{
			if ( msg->mask[i] == 1 ) thruster_vals_[i] = msg->motors[i];
		}
	}

	bool resetPoseCB( std_srvs::Empty::Request & req,
	                  std_srvs::Empty::Response & resp )
	{
    //initial_orientation_
		seabee_body_->getMotionState()->setWorldTransform( btTransform() );
		seabee_body_->setAngularVelocity( btVector3() );
		seabee_body_->setLinearVelocity( btVector3() );
		seabee_body_->setCenterOfMassTransform( btTransform() );
		return true;
	}

};

int main( int argc,
          char** argv )
{

	ros::init( argc,
	           argv,
	           "seabee3_physics" );
	ros::NodeHandle nh( "~" );

	Seabee3Physics physModel( nh );
	physModel.spin();

	return 0;
}
