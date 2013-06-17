/***************************************************************************
 *  include/uscauv_common/image_geometry.h
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


#ifndef USCAUV_USCAUVCOMMON_IMAGEGEOMETRY
#define USCAUV_USCAUVCOMMON_IMAGEGEOMETRY

// ROS
#include <ros/ros.h>

#include <tf/LinearMath/Transform.h>
#include <image_geometry/pinhole_camera_model.h>
#include <opencv2/core/core.hpp>

namespace uscauv
{
  
  /** 
   * Take a matched object and estimate its 3d position relative to the camera frame
   * based on a-priori knowledge of the object's physical size
   * 
   * @param model Camera model
   * @param center Coordinates of the matched object's center in the camera image
   * @param radius_pixels Radius of the matched object in the camera image
   * @param radius_meters Object's physical radius, in meters
   * 
   * @return Vector from the center of the camera to the object's center
   */
  tf::Vector3 reprojectObjectTo3d( image_geometry::PinholeCameraModel const & model,
				   cv::Point2d const & center, double const &radius_pixels,
				   double const & radius_meters );
  
    
} // uscauv

#endif // USCAUV_USCAUVCOMMON_IMAGEGEOMETRY
