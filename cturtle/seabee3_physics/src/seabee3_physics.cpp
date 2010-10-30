/*******************************************************************************
 *
 *      seabee3_physics
 * 
 *      Copyright (c) 2010, Randolph C. Voorhies (voorhies at usc dot edu)
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
#include <seabee3_beestem/BeeStem3.h>
#include <seabee3_driver_base/MotorCntl.h>



class Seabee3Physics : public BaseTfTranceiver<>
{
  public:
    Seabee3Physics(ros::NodeHandle &nh) :
      BaseTfTranceiver<>(nh),
      num_thrusters_(6),
      thruster_transforms_(num_thrusters_),
      thruster_transform_name_prefix_("/seabee3/thruster"),
      rate_(60),
      thruster_vals_(num_thrusters_)
  {

    motor_cntl_sub_ = nh.subscribe( "/seabee3/motor_cntl", 1, &Seabee3Physics::motorCntlCB, this);

    updateThrusterTransforms();

    // Build the broadphase
    broadphase_ = new btDbvtBroadphase();

    // Set up the collision configuration and dispatcher
    collision_configuration_ = new btDefaultCollisionConfiguration();
    dispatcher_ = new btCollisionDispatcher(collision_configuration_);

    // The actual physics solver
    solver_ = new btSequentialImpulseConstraintSolver;

    // The world
    dynamics_world_ =
      new btDiscreteDynamicsWorld(dispatcher_,broadphase_,solver_,collision_configuration_);
    dynamics_world_->setGravity(btVector3(0,0,-9.81));

    // Seabee's Body
    seabee_shape_ = new btCylinderShape(btVector3(.5, .2, .2));

    seabee_motion_state_ =
      new btDefaultMotionState(btTransform(btQuaternion(0,0,0,1), btVector3(0,0,0)));

    // She weighs 35kg
    btScalar seabee_mass = 35;
    btVector3 seabee_inertia(0,0,0);
    seabee_shape_->calculateLocalInertia(seabee_mass, seabee_inertia);

    btRigidBody::btRigidBodyConstructionInfo seabee_body_ci(
        seabee_mass, seabee_motion_state_, seabee_shape_, seabee_inertia);
    seabee_body_ = new btRigidBody(seabee_body_ci);
    dynamics_world_->addRigidBody(seabee_body_);
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

      // Step the physics simulation
      dynamics_world_->stepSimulation(1.0/rate_, 10);

      btTransform trans;
      seabee_body_->getMotionState()->getWorldTransform(trans);

      ROS_INFO("Body Height: %f", trans.getOrigin().getY());

      publishTfFrame(trans, "/landmark_map", "/seabee3/base_link");
      ros::Rate(rate_).sleep();
    }

    void updateThrusterTransforms()
    {
      // Fill in all of our thruster transforms
      for(size_t i=0; i<thruster_transforms_.size(); ++i)
      {
        std::ostringstream stream;
        stream << thruster_transform_name_prefix_ << i;
        tf::Transform tmp_transform;
        fetchTfFrame(tmp_transform, "/seabee3/base_link", stream.str());
        tmp_transform >> thruster_transforms_[i];
      }
    }

    void motorCntlCB(const seabee3_driver_base::MotorCntlConstPtr &msg)
    {
      for(size_t i=0; i<msg->motors.size(); ++i)
        if(msg->mask[i])
          thruster_vals_[i] = msg->motors[i];
    }

  private:
    size_t num_thrusters_;

    std::vector<geometry_msgs::Twist> thruster_transforms_;
    std::string thruster_transform_name_prefix_;

    // The update rate
    float rate_;

    // Bullet related stuff
    btBroadphaseInterface                 *broadphase_;
    btDefaultCollisionConfiguration       *collision_configuration_;
    btCollisionDispatcher                 *dispatcher_;
    btSequentialImpulseConstraintSolver   *solver_;
    btDiscreteDynamicsWorld               *dynamics_world_;
    btCollisionShape                      *seabee_shape_;
    btDefaultMotionState                  *seabee_motion_state_;
    btRigidBody                           *seabee_body_;

    ros::Subscriber motor_cntl_sub_;
    std::vector<int> thruster_vals_;

};


int main(int argc, char** argv)
{

  ros::init(argc, argv, "seabee3_physics");
  ros::NodeHandle nh;

  
  Seabee3Physics physModel(nh);

  physModel.spin( SpinModeId::loop_spin_once );

  /*



  ROS_INFO("HELLO");




  */

  return 0;
}
