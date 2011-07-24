/*******************************************************************************
 *
 *      imu_integrator
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
#include <std_srvs/Empty.h>
#include <xsens_node/Imu.h>


class IMUIntegrator: public BaseTfTranceiver<>
{
private:

	// Bullet related stuff
	btBroadphaseInterface *broadphase_;
	btDefaultCollisionConfiguration *collision_configuration_;
	btCollisionDispatcher *dispatcher_;
	btSequentialImpulseConstraintSolver *solver_;
	btDiscreteDynamicsWorld *dynamics_world_;
	btCollisionShape *seabee_shape_;
	btDefaultMotionState *seabee_motion_state_;
	btRigidBody *seabee_body_;

	ros::Subscriber imu_sub_;
  ros::ServiceServer reset_pose_svr_;

	ros::Time last_call_;

public:
	IMUIntegrator( ros::NodeHandle &nh ) : BaseTfTranceiver<>( nh )	
  {
		reset_pose_svr_ = nh_local_.advertiseService( "reset_pose", &IMUIntegrator::resetPoseCB, this );

		ros::Duration( 0.5 ).sleep(); //wait for this frame to register on the network

		publishTfFrame( tf::Transform( tf::Quaternion( 0, 0, 0 ), tf::Vector3( 0, 0, 0 ) ), "/landmark_map", "/seabee3/imu_integrator" );

		imu_sub_ = nh.subscribe( "/xsens/custom_data", 1, &IMUIntegrator::imuCB, this );

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

	~IMUIntegrator()
	{
		delete dynamics_world_;
		delete solver_;
		delete dispatcher_;
		delete collision_configuration_;
		delete broadphase_;
		delete seabee_motion_state_;
		delete seabee_body_;
	}

	void imuCB( const xsens_node::Imu::ConstPtr & imu_msg )
	{
    btTransform world_transform;

    // Snap the orientation and the given values
    seabee_body_->getMotionState()->getWorldTransform( world_transform );
    world_transform.setRotation(btQuaternion(imu_msg->ori.z, imu_msg->ori.y, imu_msg->ori.x));
    seabee_body_->getMotionState()->setWorldTransform( world_transform );

    // Rotate the accelerations to be inline with the orientation
    btVector3 raw_accel(imu_msg->accel.x, imu_msg->accel.y, imu_msg->accel.z);
    btTransform imu_transform(btQuaternion(imu_msg->ori.z, imu_msg->ori.y, imu_msg->ori.x));
    btVector3 rotated_accel = imu_transform(raw_accel);

    //btVector3 norm_accels = raw_accels.normalized();

    ROS_INFO("Raw Accel:     %f, %f, %f", raw_accel.x(), raw_accel.y(), raw_accel.z());
    ROS_INFO("Rotated Accel: %f, %f, %f", rotated_accel.x(), rotated_accel.y(), rotated_accel.z());

//    // Set the accelerations as 'gravity'
//    seabee_body_->setGravity(aligned_accels);
//
//    // Simulate
//    double dt = ( ros::Time::now() - last_call_ ).toSec();
//    dynamics_world_->stepSimulation( dt, 50, 1.0 / rate_ );
//    last_call_ = ros::Time::now();
//
//    // Snap the orientation and depth to the given values again
//    seabee_body_->getMotionState()->getWorldTransform( world_transform );
//    world_transform.setRotation( imu_msg.ori.z );
//    seabee_body_->getMotionState()->setWorldTransform( world_transform );
//
//    // Publish
//    publishTfFrame( world_transform,
//        "/landmark_map",
//        "/seabee3/imu_integrator" );
	}

	void spinOnce()
	{ }

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
	           "imu_integrator" );
	ros::NodeHandle nh( "~" );

	IMUIntegrator physModel( nh );
	physModel.spin();

	return 0;
}

