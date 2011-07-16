/*******************************************************************************
 *
 *      transdec_localizer
 *
 *      Copyright (c) 2010
 *
 *      Randolph Voorhies
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
 *      * Neither the name of "seabee3-ros-pkg" nor the names of its
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

#include <base_image_proc/base_image_proc.h>
#include <transdec_localizer/TransdecLocalizerConfig.h>
#include <common_utils/tf.h>

typedef BaseImageProc<transdec_localizer::TransdecLocalizerConfig, std_srvs::Empty> _BaseImageProc;

class TransdecLocalizer: public _BaseImageProc
{
  public:
    // ######################################################################
    TransdecLocalizer(ros::NodeHandle & nh) : _BaseImageProc(nh)
    { 
      initCfgParams(); 
    }

    // ######################################################################
    IplImage* processImage( IplImage * ipl_img, _ServiceRequest & req, _ServiceResponse & resp )
    { 
      int minRadius = reconfigure_params_.minRadius;
      int maxRadius = reconfigure_params_.maxRadius;
      double minDist = reconfigure_params_.minDist;
      int cannythresh1 = reconfigure_params_.cannythresh1;
      int cannythresh2 = reconfigure_params_.cannythresh2;
      int houghthresh1 = reconfigure_params_.houghthresh1;
      int houghthresh2 = reconfigure_params_.houghthresh2;
      bool showDebug = reconfigure_params_.showDebug;
      double pixelsPerMeter = reconfigure_params_.pixelsPerMeter;

      cv::Mat mat(ipl_img);
      std::vector<cv::Vec3f> circles;

      // Find the circle(s)
      cv::Canny(mat, mat, cannythresh1, cannythresh2);
      cv::HoughCircles(mat, circles, CV_HOUGH_GRADIENT, 1, minDist, houghthresh1, houghthresh2, minRadius, maxRadius);

      if(circles.size() > 0)
      {
        tf::Transform current_pose_tf_;
        geometry_msgs::Twist current_pose;
        tf_utils::fetchTfFrame( current_pose_tf_, "/landmark_map", "seabee3/base_link" );
        current_pose_tf_ >> current_pose;

        geometry_msgs::Twist localized_pose = current_pose;
        double x = circles[0][0];
        double y = circles[0][1];
        tf::Transform robot_to_center_tf;
        robot_to_center_tf.setOrigin(x,y,0);
        robot_to_center_tf.setRotation(createQuaternionFromRPY(0, 0, current_pose.yaw));


        tf::Transform localized_pose_tf;
        localized_pose >> localized_pose_tf;
        tf_utils::publishTfFrame( localized_pose_tf, "/landmark_map", "/seabee3/desired_pose" );
      }

      ROS_INFO("Found %lu circles", circles.size());
      if(showDebug)
      {
        for(size_t i=0; i<circles.size(); ++i)
        {
          cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
          int radius = cvRound(circles[i][2]);
          cv::circle(mat, center, radius, cv::Scalar(255), 3, 8, 0);
          cv::line(mat, center+cv::Point(-radius,0), center+cv::Point(radius,0), cv::Scalar(255),2);
          cv::line(mat, center+cv::Point(0,-radius), center+cv::Point(0,radius), cv::Scalar(255),2);
        }
      }

      return ipl_img;
    }

    // ######################################################################
    void reconfigureCB( _ReconfigureType &config, uint32_t level )
    { }
};

int main( int argc, char **argv )
{
	ros::init( argc, argv, "transdec_localizer" );
	ROS_INFO("Starting up...");
	ros::NodeHandle nh("~");

	TransdecLocalizer transdec_localizer( nh );
	ROS_INFO("Ready to receive requests");
	transdec_localizer.spin();

	return 0;
}

