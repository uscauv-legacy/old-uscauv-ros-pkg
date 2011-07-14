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
#include <tf/transform_listener.h>
#include "btBulletDynamicsCommon.h"
#include <base_tf_tranceiver/base_tf_tranceiver.h>
#include <vector>
#include <sstream>
#include <seabee3_common/movement_common.h>
#include <seabee3_driver_base/MotorCntl.h>
#include <seabee3_common/PhysicsState.h>

class Seabee3Physics: public BaseTfTranceiver<>
{
public:
	typedef movement_common::MotorControllerIDs _MotorControllerIDs;
	const static int _NUM_MOTOR_CONTROLLERS = movement_common::NUM_MOTOR_CONTROLLERS;

private:
	std::vector<geometry_msgs::Twist> thruster_transforms_;
	std::string thruster_transform_name_prefix_;

	// The update rate
	float rate_;

	// Bullet related stuff
	btBroadphaseInterface *broadphase_;
	btDefaultCollisionConfiguration *collision_configuration_;
	btCollisionDispatcher *dispatcher_;
	btSequentialImpulseConstraintSolver *solver_;
	btDiscreteDynamicsWorld *dynamics_world_;
	btCollisionShape *seabee_shape_;
	btDefaultMotionState *seabee_motion_state_;
	btRigidBody *seabee_body_;

	ros::Subscriber motor_cntl_sub_;
	ros::Publisher physics_state_pub_;
	std::vector<int> thruster_vals_;

	ros::Time last_call_;

public:
	Seabee3Physics( ros::NodeHandle &nh ) :
		BaseTfTranceiver<> ( nh ), thruster_transforms_( _NUM_MOTOR_CONTROLLERS ), thruster_transform_name_prefix_( "/seabee3/thruster" ), rate_( 60 ), thruster_vals_(
				_NUM_MOTOR_CONTROLLERS )
	{
		publishTfFrame( tf::Transform( tf::Quaternion( 0, 0, 0 ), tf::Vector3( 0, 0, 0 ) ), "/landmark_map", "/seabee3/base_link" );

		ros::Duration( 0.5 ).sleep(); //wait for this frame to register on the network

		updateThrusterTransforms();

		motor_cntl_sub_ = nh.subscribe( "/seabee3/motor_cntl", 1, &Seabee3Physics::motorCntlCB, this );
		physics_state_pub_ = nh.advertise<seabee3_common::PhysicsState> ( std::string( "/seabee3/physics_state" ), 1 );


		// Build the broadphase
		broadphase_ = new btDbvtBroadphase();


		// Set up the collision configuration and dispatcher
		collision_configuration_ = new btDefaultCollisionConfiguration();
		dispatcher_ = new btCollisionDispatcher( collision_configuration_ );


		// The actual physics solver
		solver_ = new btSequentialImpulseConstraintSolver;


		// The world
		dynamics_world_ = new btDiscreteDynamicsWorld( dispatcher_, broadphase_, solver_, collision_configuration_ );
		dynamics_world_->setGravity( btVector3( 0, 0, 0 ) );


		// Seabee's Body
		seabee_shape_ = new btCylinderShape( btVector3( .5, .2, .2 ) );

		seabee_motion_state_ = new btDefaultMotionState( btTransform( btQuaternion( 0, 0, 0, 1 ), btVector3( 0, 0, 0 ) ) );


		// She weighs 35kg
		btScalar seabee_mass = 100;
		btVector3 seabee_inertia( 0, 0, 0 );
		seabee_shape_->calculateLocalInertia( seabee_mass, seabee_inertia );

		btRigidBody::btRigidBodyConstructionInfo seabee_body_ci( seabee_mass, seabee_motion_state_, seabee_shape_, seabee_inertia );
		seabee_body_ = new btRigidBody( seabee_body_ci );
		seabee_body_->setActivationState( DISABLE_DEACTIVATION );
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

	void spinOnce()
	{
		//updateThrusterTransforms();

		seabee_body_->clearForces();

		for ( size_t i = 0; i < _NUM_MOTOR_CONTROLLERS; i++ )
		{
			if ( i == _MotorControllerIDs::DROPPER_STAGE1 || i == _MotorControllerIDs::DROPPER_STAGE2 || i == _MotorControllerIDs::SHOOTER ) continue;

			float thrust = thruster_vals_[i] * 0.05;
			geometry_msgs::Vector3 pos = thruster_transforms_[i].linear;
			geometry_msgs::Vector3 ori = thruster_transforms_[i].angular;

			btVector3 force;
			force.setY( thrust );


			// Y P R
			btTransform force_tf( btQuaternion( ori.z, ori.y, ori.x ) );

			btVector3 rel_pos;
			rel_pos.setX( pos.x );
			rel_pos.setY( pos.y );
			rel_pos.setZ( pos.z );


			//ROS_INFO( "MOTOR%Zu THRUST: %f (%f,%f,%f) @ (%f,%f,%f)", motorIdx, thrust, force.x(), force.y(), force.z(), rel_pos.x(), rel_pos.y(), rel_pos.z() );

			seabee_body_->applyForce( force_tf * force, rel_pos );
		}

		double dt = ( ros::Time::now() - last_call_ ).toSec();


		// Step the physics simulation
		dynamics_world_->stepSimulation( dt, 50, 1.0 / rate_ );
		last_call_ = ros::Time::now();

		btTransform trans;
		seabee_body_->getMotionState()->getWorldTransform( trans );


		//printf( "Body Pos: %f %f %f\n", trans.getOrigin().getX(), trans.getOrigin().getY(), trans.getOrigin().getZ() );

		publishTfFrame( trans, "/landmark_map", "/seabee3/base_link" );

		seabee3_common::PhysicsState physics_state_msg;
		physics_state_msg.mass.linear.x = physics_state_msg.mass.linear.y = physics_state_msg.mass.linear.z = 100;
		physics_state_msg.mass.angular.x = seabee_body_->getInvInertiaTensorWorld()[0][0];
		physics_state_msg.mass.angular.y = seabee_body_->getInvInertiaTensorWorld()[1][1];
		physics_state_msg.mass.angular.z = seabee_body_->getInvInertiaTensorWorld()[2][2];

		btVector3 forces = seabee_body_->getTotalForce();
		btVector3 torque = seabee_body_->getTotalTorque();
		//printf( "(%f %f %f) (%f %f %f)\n", forces.x(), forces.y(), forces.z(), torque.x(), torque.y(), torque.z() );

		physics_state_msg.force.linear.x = forces.x();
		physics_state_msg.force.linear.y = forces.y();
		physics_state_msg.force.linear.z = forces.z();

		physics_state_msg.force.angular.x = torque.x();
		physics_state_msg.force.angular.x = torque.y();
		physics_state_msg.force.angular.x = torque.z();

		btVector3 linear_velocity = seabee_body_->getLinearVelocity();
		btVector3 angular_velocity = seabee_body_->getAngularVelocity();

		physics_state_msg.velocity.linear.x = linear_velocity.x();
		physics_state_msg.velocity.linear.y = linear_velocity.y();
		physics_state_msg.velocity.linear.z = linear_velocity.z();

		physics_state_msg.velocity.angular.x = angular_velocity.x();
		physics_state_msg.velocity.angular.y = angular_velocity.y();
		physics_state_msg.velocity.angular.z = angular_velocity.z();

		physics_state_pub_.publish( physics_state_msg );

		ros::Rate( rate_ ).sleep();
	}

	void updateThrusterTransforms()
	{
		// Fill in all of our thruster transforms
		for ( size_t i = 0; i < _NUM_MOTOR_CONTROLLERS; ++i )
		{
			if ( i == _MotorControllerIDs::DROPPER_STAGE1 || i == _MotorControllerIDs::DROPPER_STAGE2 || i == _MotorControllerIDs::SHOOTER ) continue;
			std::ostringstream stream;
			stream << thruster_transform_name_prefix_ << i;
			tf::Transform tmp_transform;
			fetchTfFrame( tmp_transform, "/seabee3/base_link", stream.str() );
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

};

int main( int argc, char** argv )
{

	ros::init( argc, argv, "seabee3_physics" );
	ros::NodeHandle nh;

	Seabee3Physics physModel( nh );
	physModel.spin( SpinModeId::LOOP_SPIN_ONCE );

	return 0;
}
