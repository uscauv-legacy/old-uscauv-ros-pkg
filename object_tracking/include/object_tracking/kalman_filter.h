/***************************************************************************
 *  include/object_tracking/kalman_filter.h
 *  --------------------
 *
 *  Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Dylan Foster
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
 *  * Neither the name of USC AUV nor the names of its
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


#ifndef USCAUV_OBJECTTRACKING_KALMANFILTER
#define USCAUV_OBJECTTRACKING_KALMANFILTER

// ROS
#include <ros/ros.h>

/// Eigen
#include <Eigen/Dense>

namespace uscauv
{

  /**
   * A linear Kalman filter for which the dimensions of the state, control inputs, and measurement inputs are known at compile time.
   * Dimensions can also be set to Eigen::Dynamic
   */

  template<unsigned int __StateDim, 
    unsigned int __ControlDim = __StateDim, unsigned int __UpdateDim = __StateDim>
    class LinearKalmanFilter
    {
    public:
    typedef Eigen::Matrix<double, __StateDim, 1>              StateVector;
    typedef Eigen::Matrix<double, __ControlDim, 1>            ControlVector;
    typedef Eigen::Matrix<double, __UpdateDim, 1>             UpdateVector;
    typedef Eigen::Matrix<double, __StateDim, __StateDim>     StateMatrix;
    typedef Eigen::Matrix<double, __ControlDim, __ControlDim> ControlMatrix;
    typedef Eigen::Matrix<double, __StateDim, __StateDim>     UpdateMatrix;
    typedef Eigen::Matrix<double, __StateDim, __ControlDim>   StateControlMatrix;
    typedef Eigen::Matrix<double, __UpdateDim, __StateDim>    UpdateStateMatrix;
 
    private:
    /// state vector
    StateVector state_;
    /// state covariance
    StateMatrix cov_;

    /// See LKF definition in Probabilistic Robotics
    const StateMatrix        A_;
    const StateControlMatrix B_;
    const UpdateStateMatrix  C_;

    public:
    /// Using Identity automatrically even when dims aren't the same is a little iffy
    LinearKalmanFilter(StateVector const & init_state, StateMatrix const &init_cov, 
		       StateMatrix        const & A = StateMatrix::Identity(), 
		       StateControlMatrix const & B = StateControlMatrix::Identity(),
		       UpdateStateMatrix  const & C = UpdateStateMatrix::Identity()
		       )
    : state_(init_state), cov_(init_cov),
    A_(A), B_(B), C_(C)
    {}
    
    void reset( StateVector const & init_state, StateMatrix const & init_cov )
    {
      state_ = init_state; cov_ = init_cov;
    }
    
    private:
    void predict( ControlVector const & control, ControlMatrix const & control_cov )
    {
      state_ = A_*state_ + B_*control;
      cov_ = A_* cov_.inverse() * A_.transpose() + control_cov;
    }

    void update( UpdateVector const & update, UpdateMatrix const & update_cov )
    {
      UpdateMatrix gain = cov_*C_.transpose() * (C_*cov_*C_.transpose() + update_cov).inverse();
      state_ = state_ + gain*( update - state_ );
      cov_ = ( StateMatrix::Identity() - gain*C_)*cov_;
    }
 
    };
  
}

#endif // USCAUV_OBJECTTRACKING_KALMANFILTER
