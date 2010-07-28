/*********************************************************************
 * Software License Agreement (BSD License)
 * 
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 * 
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 * 
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 * 
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/CvBridge.h>

#include <image_transport/image_transport.h>
#include <seabee3_driver_base/Pressure.h>

#include <boost/thread.hpp>
#include <boost/format.hpp>
#include <boost/lexical_cast.hpp>

using namespace cv;

class Dashboard
{
private:
  image_transport::Subscriber image_sub_;
  ros::Subscriber extl_pressure_sub_;
  
  sensor_msgs::ImageConstPtr last_msg_;
  sensor_msgs::CvBridge img_bridge_;
  boost::mutex image_mutex_;
  boost::mutex pressure_mutex_;
  IplImage* dash_img_;
  std::string window_name_;
  boost::format filename_format_;
  int count_;
  int extl_pressure_;
public:
  Dashboard(const ros::NodeHandle& nh, const std::string& transport)
    : filename_format_(""), count_(0)
  {
    std::string topic = nh.resolveName("image");
    ros::NodeHandle local_nh("~");
    local_nh.param("window_name", window_name_, std::string("Seabee3 Dasboard"));

    bool autosize;
    local_nh.param("autosize", autosize, false);
    
    std::string format_string;
    local_nh.param("filename_format", format_string, std::string("frame%04i.jpg"));
    filename_format_.parse(format_string);

    dash_img_ = cvCreateImage(cvSize(640,480),IPL_DEPTH_32F,3);

    cvNamedWindow(window_name_.c_str(), 0);
    cvResizeWindow(window_name_.c_str(), 640, 480);
    cvShowImage(window_name_.c_str(), dash_img_);

    cvStartWindowThread();

    image_transport::ImageTransport it(nh);

    ROS_INFO("transport: %s",transport.c_str());

    image_sub_ = it.subscribe(topic, 1, &Dashboard::image_cb, this, transport);
    extl_pressure_sub_ = local_nh.subscribe(std::string("/seabee3/extl_pressure"), 1, &Dashboard::extl_pressure_cb, this);
  }

  ~Dashboard()
  {
    cvReleaseImage(&dash_img_);
    cvDestroyWindow(window_name_.c_str());
  }

  void renderDepth()
  {
    boost::lock_guard<boost::mutex> guard(pressure_mutex_);

    int itsCurrentDepth = extl_pressure_;
    ROS_INFO("depth: %d",itsCurrentDepth);
    int minDrawDepth = itsCurrentDepth - 100;

    int drawHeight = (int)(.3*(float)dash_img_->height);
    int maxDrawHeight = drawHeight + 200;

    // draw tick marks
    while(drawHeight < maxDrawHeight)
      {	
	if((minDrawDepth %100 ) % 13 == 0)
	  {
	    cvLine(dash_img_,
		   cvPoint((int)(.48*(float)dash_img_->width),(int)drawHeight),
		   cvPoint((int)(.52*(float)dash_img_->width),(int)drawHeight),
		   CV_RGB(20,253,0)//CV_RGB( 100, 100, 100 )
		   );
	  }

	if(minDrawDepth % 25 == 0)
	  {
	    if(minDrawDepth % 100 == 0 ||
	       minDrawDepth % 100 == 50)
	      {
		cvLine(dash_img_, cvPoint((int)(.35*(float)dash_img_->width),drawHeight),
		       cvPoint((int)(.65*(float)dash_img_->width),drawHeight),
		       CV_RGB(20,253,0)//CV_RGB(200,200,200)
		       );
	      }
	    else if(minDrawDepth % 100 == 25 ||
		    minDrawDepth % 100 == 75)
	      {
		CvFont font;
		double hScale=0.5;
		double vScale=0.5;
		int lineWidth=2;

		cvInitFont(&font,CV_FONT_HERSHEY_SIMPLEX, hScale,vScale,0,lineWidth);

		cvPutText(dash_img_,
			  boost::lexical_cast<std::string>(minDrawDepth).c_str(),
			  cvPoint((int)(.60*(float)dash_img_->width + 8),drawHeight-12),
			  &font,
			  //			8.0,
			  CV_RGB(20,253,0)
			  );
		
		cvLine(dash_img_,
		       cvPoint((int)(.35*(float)dash_img_->width),drawHeight),
		       cvPoint((int)(.65*(float)dash_img_->width),drawHeight),
		       CV_RGB(20,253,0)//CV_RGB(100,100,100)
		       );
	      }

	    if(drawHeight + 13 < dash_img_->height)
	      {
		cvLine(dash_img_,
		       cvPoint((int)(.48*(float)dash_img_->width),drawHeight + 13),
		       cvPoint((int)(.52*(float)dash_img_->width),drawHeight+13),
		       CV_RGB(20,253,0)//CV_RGB(100,100,100)
		       );
	      }

	    drawHeight += 25;
	    minDrawDepth += 25;
	  }
	else
	  {
	    drawHeight++;
	    minDrawDepth++;
	  }
      }
    
    // draw current depth line as horizontal over middle of image
    cvLine(dash_img_, cvPoint((int)(.35*(float)dash_img_->width),
			      (int)(dash_img_->height/2-2)), 
	   cvPoint((int)(.65*(float)dash_img_->width),
		   (dash_img_->height/2 - 2)),
	   CV_RGB(200,200,200));
    cvLine(dash_img_, cvPoint((int)(.35*(float)dash_img_->width),(dash_img_->height/2+2)), cvPoint((int)(.65*(float)dash_img_->width),(dash_img_->height/2 + 2)), CV_RGB(200,200,200));		 
  }
  
  void image_cb(const sensor_msgs::ImageConstPtr& msg)
  {
    boost::lock_guard<boost::mutex> guard(image_mutex_);
    last_msg_ = msg;
    
    // May want to view raw bayer data
    // NB: This is hacky, but should be OK since we have only one image CB.
    if (msg->encoding.find("bayer") != std::string::npos)
      boost::const_pointer_cast<sensor_msgs::Image>(msg)->encoding = "mono8";
    
    if (img_bridge_.fromImage(*msg, "bgr8"))
      {
	dash_img_ = img_bridge_.toIpl();
      }
    else
      ROS_ERROR("Unable to convert image to bgr8");
    
    renderDepth();
    
    cvShowImage(window_name_.c_str(), dash_img_);
  }

  void extl_pressure_cb(const seabee3_driver_base::PressureConstPtr& msg)
  {
    boost::lock_guard<boost::mutex> guard(pressure_mutex_);
    extl_pressure_ = msg->Value;
  }
    
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_view", ros::init_options::AnonymousName);
  ros::NodeHandle n;
  if (n.resolveName("image") == "/image") {
    ROS_WARN("image_view: image has not been remapped! Typical command-line usage:\n"
             "\t$ ./image_view image:=<image topic> [transport]");
  }

  Dashboard view(n, (argc > 1) ? argv[1] : "raw");

  ros::spin();
  
  return 0;
}

