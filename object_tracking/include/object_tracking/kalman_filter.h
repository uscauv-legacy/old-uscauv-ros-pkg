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

  template<unsigned int __StateDim, typename __NumericType = double>
    class LinearKalmanFilter
    {
    public:
    typedef Eigen::Matrix<__NumericType, __StateDim, 1>              StateVector;
    typedef Eigen::Matrix<__NumericType, __StateDim, __StateDim>     StateMatrix;

    /// TODO: Redo these using type aliases
    template<unsigned int __ControlDim>
    struct Control
    {
      typedef Eigen::Matrix<__NumericType, __ControlDim, 1>            VectorType;
      typedef Eigen::Matrix<__NumericType, __ControlDim, __ControlDim> CovarianceType;
      typedef Eigen::Matrix<__NumericType, __StateDim, __ControlDim>   TransitionType;
    };

    template<unsigned int __UpdateDim>
    struct Update
    {
      typedef Eigen::Matrix<__NumericType, __UpdateDim, 1>           VectorType;
      typedef Eigen::Matrix<__NumericType, __UpdateDim, __UpdateDim> CovarianceType;
      typedef Eigen::Matrix<__NumericType, __UpdateDim, __StateDim>  TransitionType;
      typedef Eigen::Matrix<__NumericType, __StateDim, __UpdateDim>  GainType;
    };

    public:
    /// state vector
    StateVector state_;
    /// state covariance
    StateMatrix cov_;

    /* /// See LKF definition in Probabilistic Robotics */
    /* StateMatrix        A_; */
    
    public:
    /// Using Identity automatrically even when dims aren't the same is a little iffy
    LinearKalmanFilter(StateVector const & init_state, StateMatrix const &init_cov, 
		       StateMatrix        const & A = StateMatrix::Identity()
		       )
    : state_(init_state), cov_(init_cov)/* , A_(A) */{}

    LinearKalmanFilter(): 
    state_( StateVector::Zero() ), cov_( StateMatrix::Identity() )/* , */
    /* A_( StateMatrix::Identity() ) */{}
    
    LinearKalmanFilter( LinearKalmanFilter const &src):
    state_(src.state_), cov_(src.cov_)/* , A_(src.A_) */{}
    
    
    LinearKalmanFilter& operator=( LinearKalmanFilter const & rhs)
    {
      state_ = rhs.state_;
      cov_ = rhs.cov_;
      /* A_ = rhs.A_; */
            
      return *this;
    }
    
    public:
    void reset( StateVector const & init_state, StateMatrix const & init_cov )
    {
      state_ = init_state; cov_ = init_cov;
    }
    
    template< unsigned int __ControlDim >
    void predict( typename Control<__ControlDim>::VectorType     const & control, 
		  typename Control<__ControlDim>::CovarianceType const & control_cov,
		  StateMatrix const & A,
		  typename Control<__ControlDim>::TransitionType const & B = 
		  Control<__ControlDim>::TransitionType::Zero() )
    {
      state_ = A*state_ + B*control;
      cov_ = A* cov_ * A.transpose() + control_cov;
    }
    
    template< unsigned int __UpdateDim>
    void update( typename Update<__UpdateDim>::VectorType const & update,
		 typename Update<__UpdateDim>::CovarianceType const & update_cov,
		 typename Update<__UpdateDim>::TransitionType const & C )
    {
      typename Update<__UpdateDim>::GainType gain = 
      cov_*C.transpose() * (C*cov_*C.transpose() + update_cov).inverse();
      
      state_ = state_ + gain*( update - C*state_ );
      cov_ = ( StateMatrix::Identity() - gain*C)*cov_;
    }

    /// In case you want to print the entire state 
    friend std::ostream& operator<< (std::ostream& os, 
				     LinearKalmanFilter const & kf )
    {
      os << std::endl;
      os << "State: " << std::endl << kf.state_ << std::endl;
      os << "Covariance: "<< std::endl << kf.cov_ << std::endl;
      /* os << "A: "<< std::endl << kf.A_ << std::endl; */
      return os;
    }
    
    };
 
}

#endif // USCAUV_OBJECTTRACKING_KALMANFILTER
