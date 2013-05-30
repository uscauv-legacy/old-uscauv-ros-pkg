/***************************************************************************
 *  include/uscauv_common/graphics.h
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


#ifndef USCAUV_USCAUVCOMMON_GRAPHICS
#define USCAUV_USCAUVCOMMON_GRAPHICS

// ROS
#include <ros/ros.h>

/// opencv2
#include <opencv2/core/core.hpp>

namespace uscauv
{

  // ################################################################
  // Colors! Feel free to add more ##################################
  // ################################################################

  static const cv::Scalar CV_RED_BGR         = cv::Scalar( 0  ,   0, 255 );
  static const cv::Scalar CV_BLUE_BGR        = cv::Scalar( 255, 0  , 0   );
  static const cv::Scalar CV_GREEN_BGR       = cv::Scalar( 0  , 255, 0   );
  static const cv::Scalar CV_YELLOW_BGR      = cv::Scalar( 0  , 255, 255 );
  static const cv::Scalar CV_MAGENTA_BGR     = cv::Scalar( 255, 0  , 255 );
  static const cv::Scalar CV_CYAN_BGR        = cv::Scalar( 255, 255, 0   );
  static const cv::Scalar CV_WHITE_BGR       = cv::Scalar( 255, 255, 255 );
  static const cv::Scalar CV_BLACK_BGR       = cv::Scalar( 0  , 0  , 0   );
  static const cv::Scalar CV_ORANGE_BGR      = cv::Scalar( 0  , 127, 255 );
  static const cv::Scalar CV_PINK_BGR        = cv::Scalar( 160, 120, 200 );
  static const cv::Scalar CV_LIME_BGR        = cv::Scalar( 120, 200, 160 );
  static const cv::Scalar CV_DENIM_BGR       = cv::Scalar( 189,  96, 21  );
  static const cv::Scalar CV_USCCARDINAL_BGR = cv::Scalar( 0  , 0  , 153 );
  static const cv::Scalar CV_USCGOLD_BGR     = cv::Scalar( 0  , 204, 255 );
  
  static const cv::Scalar CV_RED_RGB         = cv::Scalar( 255, 0  , 0   );
  static const cv::Scalar CV_BLUE_RGB        = cv::Scalar( 0  , 0  , 255 );
  static const cv::Scalar CV_GREEN_RGB       = cv::Scalar( 0  , 255, 0   );
  static const cv::Scalar CV_YELLOW_RGB      = cv::Scalar( 255, 255, 0   );
  static const cv::Scalar CV_MAGENTA_RGB     = cv::Scalar( 255, 0  , 255 );
  static const cv::Scalar CV_CYAN_RGB        = cv::Scalar( 0  , 255, 255 );
  static const cv::Scalar CV_WHITE_RGB       = cv::Scalar( 255, 255, 255 );
  static const cv::Scalar CV_BLACK_RGB       = cv::Scalar( 0  , 0  , 0   );
  static const cv::Scalar CV_ORANGE_RGB      = cv::Scalar( 255, 127, 0   );
  static const cv::Scalar CV_PINK_RGB        = cv::Scalar( 200, 120, 160 );
  static const cv::Scalar CV_LIME_RGB        = cv::Scalar( 160, 200, 120 );   
  static const cv::Scalar CV_DENIM_RGB       = cv::Scalar( 21,  96 , 189 );
  static const cv::Scalar CV_USCCARDINAL_RGB = cv::Scalar( 153, 0  , 0   );
  static const cv::Scalar CV_USCGOLD_RGB     = cv::Scalar( 255, 204, 0   );
}

#endif // USCAUV_USCAUVCOMMON_GRAPHICS
