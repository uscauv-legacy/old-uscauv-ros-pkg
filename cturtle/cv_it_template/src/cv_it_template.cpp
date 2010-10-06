/*******************************************************************************
 *
 *      cv_it_template
 * 
 *      Copyright (c) 2010, Edward T. Kaszubski
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

#include <ros/ros.h>
// for ImageTransport, Publisher
#include <image_transport/image_transport.h>
// for Mat
#include <opencv/cv.h>
//for CvBridge
#include <cv_bridge/CvBridge.h>
// for a ton of boost-related shit
#include <boost/thread.hpp>
// for CameraInfo
#include <sensor_msgs/CameraInfo.h>
// for pinhole camera model
#include <image_geometry/pinhole_camera_model.h>
// for the DoSomething service
#include <cv_it_template/DoSomething.h>
// for dynamic reconfigure server
#include <dynamic_reconfigure/server.h>
// for cfg code
#include <cv_it_template/CvItTemplateConfig.h>

class CvItTemplate
{
private:
	ros::NodeHandle nh_priv_;
	ros::MultiThreadedSpinner spinner_;

	//dynamic_reconfigure::Server<cv_it_template::CvItTemplateConfig> cfg_srv_;

	sensor_msgs::ImagePtr img_;
	sensor_msgs::CameraInfo info_;

	image_transport::Publisher img_pub_;
	image_transport::Subscriber img_sub_;
	image_transport::ImageTransport it_;

	ros::Subscriber info_sub_;
	ros::ServiceServer do_something_srv_;

	sensor_msgs::CvBridge bridge_;

	cv::Mat cv_img_;

	boost::mutex img_mutex_, info_mutex_, flag_mutex_;
	bool new_img_;

public:
	CvItTemplate( ros::NodeHandle & nh ) :
		nh_priv_( "~" ), spinner_( 3 ), it_( nh_priv_ ), new_img_( false )
	{
		img_sub_ = it_.subscribe( "image", 1, &CvItTemplate::imageCB, this );
		info_sub_ = nh_priv_.subscribe( "camera_info", 1, &CvItTemplate::infoCB, this );

		img_pub_ = it_.advertise( "output_image", 1 );
		do_something_srv_ = nh_priv_.advertiseService( "do_something", &CvItTemplate::doSomethingCB, this );

		dynamic_reconfigure::Server<cv_it_template::CvItTemplateConfig> srv;
		dynamic_reconfigure::Server<cv_it_template::CvItTemplateConfig>::CallbackType f = boost::bind( CvItTemplate::reconfigureCB, _1, _2 );
		srv.setCallback( f );


		//dynamic_reconfigure::Server<cv_it_template::CvItTemplateConfig>::CallbackType cfg_callback = boost::bind( &CvItTemplate::reconfigureCB, _1, _2 );
		//cfg_srv_.setCallback( cfg_callback );
	}

	~CvItTemplate()
	{
		//
	}

	static void reconfigureCB( cv_it_template::CvItTemplateConfig &config, uint32_t level )
	{
		ROS_DEBUG( "Reconfigure successful" );
		// dynamically_reconfigurable_varname = config.dynamically_reconfigurable_varname
	}

	void imageCB( const sensor_msgs::ImageConstPtr& msg )
	{
		ROS_DEBUG( "got image" );
		img_mutex_.lock();

		img_ = boost::const_pointer_cast<sensor_msgs::Image>( msg );

		img_mutex_.unlock();

		boost::lock_guard<boost::mutex> guard( flag_mutex_ );
		new_img_ = true;
	}

	void infoCB( const sensor_msgs::CameraInfoConstPtr& msg )
	{
		ROS_DEBUG( "got camera info" );
		info_mutex_.lock();

		info_ = *msg;

		info_mutex_.unlock();
	}


	// TODO: Rename this method according to the actual service request that will be used
	bool doSomethingCB( cv_it_template::DoSomething::Request & req, cv_it_template::DoSomething::Response & resp )
	{
		// **** do not edit below this line ****
		boost::lock_guard<boost::mutex> img_guard( img_mutex_ );
		cv_img_ = cv::Mat( bridge_.imgMsgToCv( img_ ) );
		// *************************************


		// process and optionally make changes to cv_img_
		// access pixels via:
		/*for ( int y = 0; y < cv_img_.size().height; y++ )
		{
			for ( int x = 0; x < cv_img_.size().width; x++ )
			{
				// get pixel data
				cv::Vec3b data = cv_img_.at<cv::Vec3b> ( cv::Point( x, y ) );
				// set pixel data
				cv_img_.at<cv::Vec3b> ( cv::Point( x, y ) ) = data;
			}
		}*/

		// **** do not edit below this line ****
		IplImage * ipl_img = & ( (IplImage) cv_img_ );
		img_pub_.publish( bridge_.cvToImgMsg( ipl_img ) );
		// *************************************

		// notify ROS whether or not everything executed properly
		return true;
	}

	void spin()
	{
		spinner_.spin();
	}

};

int main( int argc, char ** argv )
{
	ros::init( argc, argv, "cv_it_template" );
	ros::NodeHandle nh;

	CvItTemplate cv_it_template( nh );
	cv_it_template.spin();

	return 0;
}
