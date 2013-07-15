/***************************************************************************
 *  include/auv_controls/controller.h
 *  --------------------
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

#include <ros/ros.h>

#include <array>

#include <auv_controls/pid.h>
#include <Eigen/Dense>

namespace uscauv
{

  template <class __ControlType, unsigned int __Dim>
    class ControllerND
  {
  private:
    typedef std::array<__ControlType, __Dim> _ControllerArray;
  
    _ControllerArray controllers_;

  public:
  
    ControllerND()
      {
      }

  private:
    void init_recursive(unsigned int const & idx){}

    template<class __Arg, class ...__Args>
      void init_recursive(unsigned int const & idx, __Arg const & arg, __Args && ...args)
    {
      controllers_.at( idx ).init( arg );
      init_recursive( idx + 1, args... );
    }

  protected:
    /// starts up all of the controllers
    template <class...__Args>
      typename std::enable_if< sizeof...(__Args) == __Dim, void>::type init(__Args && ...args)
    {
      init_recursive(0, args...);
      return ;
    }
	
  public:	
    /** 
     * Set the measured value of the quantity to be controlled
     *
     * @param value The new value of the measurement
     * 
     */
    template <unsigned int __Idx>
      typename std::enable_if<(__Idx < __Dim), void>::type setObserved(double const & value)
    {
      controllers_.at(__Idx).setObserved(value);
    }

    /** 
     * Get the controller's current error term
     * 
     * @return Error term
     */
    template <unsigned int __Idx>
      typename std::enable_if<(__Idx < __Dim), double>::type update()
    {
      return controllers_.at(__Idx).update();
    }

    /* /\**  */
    /*  * Update all individual controllers */
    /*  *  */
    /*  *\/ */
    /* void updateAll() */
    /* { */
    /*   for(typename _ControllerArray::iterator controller_it = controllers_.begin(); */
    /* 	  controller_it != controllers_.end(); ++controller_it) */
    /* 	{ */
    /* 	  controller_it->update(); */
    /* 	} */
    /* } */

    /** 
     * Set the controller setpoint.
     * 
     * @param value Target value for the quantity to be controller
     * 
     */
    template <unsigned int __Idx>
      typename std::enable_if<(__Idx < __Dim), void>::type setSetpoint(double const & value)
    {
      controllers_.at(__Idx).setSetpoint(value);
    }
    
  };
  
  class PID6D: public ControllerND< uscauv::PID1D, 6 >
    {
    public:

      enum Axes
      { SURGE = 0, SWAY = 1, HEAVE = 2,
	YAW = 3,   PITCH = 4,  ROLL = 5 };

      void loadController()
      {
	init("linear/x", "linear/y", "linear/z",
	     "angular/yaw", "angular/pitch", "angular/roll");
      }

      Eigen::Matrix<double, 6, 1> updateAllPID()
	{
	  Eigen::Matrix<double, 6, 1> output;

	  output(0) = update<SURGE>();
	  output(1) = update<SWAY>();
	  output(2) = update<HEAVE>();
	  output(3) = update<ROLL>();
	  output(4) = update<PITCH>();
	  output(5) = update<YAW>();
	  
	  return output;
	}
      
    };

} // uscauv
