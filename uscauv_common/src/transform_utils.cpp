/***************************************************************************
 *  src/transform_utils.cpp
 *  --------------------
 *
 *  Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Dylan Foster (turtlecannon@gmail.com)
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


#include <uscauv_common/transform_utils.h>

namespace uscauv
{

  bool isNan( tf::Vector3 const & vec)
  {
    return ( std::isnan(vec.getX()) || std::isnan(vec.getY()) || std::isnan(vec.getZ()) );
  }
  bool isNan( tf::Quaternion const & quat)
  {
    return ( std::isnan(quat.getX()) || std::isnan(quat.getY()) || 
	     std::isnan(quat.getZ()) || std::isnan(quat.getW()) );
  }
  bool isNan( tf::Transform const & transform)
  {
    return ( isNan( transform.getOrigin() ) || isNan( transform.getRotation()) );
  }
  

  bool isZero( tf::Quaternion const & quat)
  {
    return tfFuzzyZero( quat.length() );
  }

  bool isValid( tf::Vector3 const & vec)
  {
    return !isNan( vec );
  }
  bool isValid( tf::Quaternion const & quat)
  {
    return !( isNan( quat ) || isZero( quat ) );
  }
  bool isValid( tf::Transform const & transform)
  {
    return isValid( transform.getOrigin() ) && isValid( transform.getRotation() );
  }

}
