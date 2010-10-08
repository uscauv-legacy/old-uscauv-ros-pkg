/*******************************************************************************
 *
 *      color_classifier
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
// for dynamic reconfigure server
#include <dynamic_reconfigure/server.h>
// for cfg code
#include <color_classifier/ColorClassifierConfig.h>

class ColorClassifier
{
public:
	struct OutputColorRGB
	{
		const static cv::Vec3b red, orange, yellow, green, blue, black, white, unknown;
	};

	template<class _pixel_type = uchar>
	struct DataRange
	{
		_pixel_type min, max;

		bool isInRange( _pixel_type value, bool data_wraps = false )
		{
			return ( value >= min && value < max ) || ( data_wraps && min > max && ( value >= min || value < max ) );
		}
	};

	template<class _pixel_type = uchar>
	struct ColorRange
	{
		ColorRange<_pixel_type> i, j, k;
	};

	struct ColorSpectrumHSV
	{
		DataRange<double> red, orange, yellow, green, blue;
		double black_v_threshold, white_combined_sv_threshold, black_combined_sv_threshold;
	};
private:
	ros::NodeHandle nh_priv_;
	ros::MultiThreadedSpinner spinner_;

	sensor_msgs::ImagePtr img_;
	sensor_msgs::CameraInfo info_;

	image_transport::Publisher img_pub_;

	image_transport::Subscriber img_sub_;
	image_transport::ImageTransport it_;

	ros::Subscriber info_sub_;

	sensor_msgs::CvBridge bridge_;

	dynamic_reconfigure::Server<color_classifier::ColorClassifierConfig> reconfigure_srv_;
	dynamic_reconfigure::Server<color_classifier::ColorClassifierConfig>::CallbackType reconfigure_callback_;

	cv::Mat cv_img_;

	boost::mutex img_mutex_, info_mutex_, flag_mutex_;
	bool new_img_;
	std::string image_transport_;

	ColorClassifier::ColorSpectrumHSV spectrum_hsv_;

public:

	ColorClassifier( ros::NodeHandle & nh ) :
		nh_priv_( "~" ), spinner_( 3 ), it_( nh ), new_img_( false )
	{
		ROS_INFO( "Constructing..." );
		nh_priv_.param( "image_transport", image_transport_, std::string( "raw" ) );

		img_sub_ = it_.subscribe( nh.resolveName( "image" ), 1, &ColorClassifier::imageCB, this );
		info_sub_ = nh_priv_.subscribe( "camera_info", 1, &ColorClassifier::infoCB, this );

		img_pub_ = it_.advertise( "output_image", 1 );

		reconfigure_callback_ = boost::bind( &ColorClassifier::reconfigureCB, this, _1, _2 );
		reconfigure_srv_.setCallback( reconfigure_callback_ );
	}

	~ColorClassifier()
	{
		//
	}

	double wrapValue( double value, double min, double max )
	{
		while ( value < min )
		{
			value += ( max - min );
		}
		while ( value > max )
		{
			value -= ( max - min );
		}
		return value;
	}

	void reconfigureCB( color_classifier::ColorClassifierConfig &config, uint32_t level )
	{
		ROS_INFO( "Reconfigure successful" );

		spectrum_hsv_.red.min = wrapValue( config.red_h_c - config.red_h_r, 0.0, 360.0 );
		spectrum_hsv_.red.max = wrapValue( config.red_h_c + config.red_h_r, 0.0, 360.0 );

		spectrum_hsv_.orange.min = wrapValue( config.orange_h_c - config.orange_h_r, 0.0, 360.0 );
		spectrum_hsv_.orange.max = wrapValue( config.orange_h_c + config.orange_h_r, 0.0, 360.0 );

		spectrum_hsv_.yellow.min = wrapValue( config.yellow_h_c - config.yellow_h_r, 0.0, 360.0 );
		spectrum_hsv_.yellow.max = wrapValue( config.yellow_h_c + config.yellow_h_r, 0.0, 360.0 );

		spectrum_hsv_.green.min = wrapValue( config.green_h_c - config.green_h_r, 0.0, 360.0 );
		spectrum_hsv_.green.max = wrapValue( config.green_h_c + config.green_h_r, 0.0, 360.0 );

		spectrum_hsv_.blue.min = wrapValue( config.blue_h_c - config.blue_h_r, 0.0, 360.0 );
		spectrum_hsv_.blue.max = wrapValue( config.blue_h_c + config.blue_h_r, 0.0, 360.0 );


		/*spectrum_hsv_.red.min = config.red_h_min;
		 spectrum_hsv_.red.max = config.red_h_max;

		 spectrum_hsv_.orange.min = config.orange_h_min;
		 spectrum_hsv_.orange.max = config.orange_h_max;

		 spectrum_hsv_.yellow.min = config.yellow_h_min;
		 spectrum_hsv_.yellow.max = config.yellow_h_max;

		 spectrum_hsv_.green.min = config.green_h_min;
		 spectrum_hsv_.green.max = config.green_h_max;

		 spectrum_hsv_.blue.min = config.blue_h_min;
		 spectrum_hsv_.blue.max = config.blue_h_max;*/

		spectrum_hsv_.black_v_threshold = config.black_v_threshold;
		spectrum_hsv_.white_combined_sv_threshold = config.white_combined_sv_threshold;
		spectrum_hsv_.black_combined_sv_threshold = config.black_combined_sv_threshold;
	}

	void imageCB( const sensor_msgs::ImageConstPtr& msg )
	{
		ROS_INFO( "got image" );
		img_mutex_.lock();

		img_ = boost::const_pointer_cast<sensor_msgs::Image>( msg );

		img_mutex_.unlock();

		boost::lock_guard<boost::mutex> guard( flag_mutex_ );
		new_img_ = true;

		classifyImage();
	}

	void infoCB( const sensor_msgs::CameraInfoConstPtr& msg )
	{
		ROS_INFO( "got camera info" );
		info_mutex_.lock();

		info_ = *msg;

		info_mutex_.unlock();
	}

	cv::Vec3b filterColor( cv::Vec3b & pixel )
	{
		cv::Vec3b result;
		//result[2] = (int) round( (double) pixel[2] * 0.937254902 );
		//result[1] = (int) round( (double) pixel[1] * 0.635294118 );
		//result[0] = (int) round( (double) pixel[0] * 0.521568627 );
		result[2] = std::max( pixel[2] - 16, 0 );
		result[1] = std::max( pixel[1] - 93, 0 );
		result[0] = std::max( pixel[0] - 122, 0 );
		return result;
	}

	cv::Vec3b classifyPixel( cv::Vec3b & pixel )
	{
		double h = 360.0 * (double) pixel[0] / 255.0;
		double l = 100.0 * (double) pixel[1] / 255.0;
		double s = 100.0 * (double) pixel[2] / 255.0;

		if ( l < spectrum_hsv_.black_v_threshold ) return OutputColorRGB::black;
		if ( l > spectrum_hsv_.white_combined_sv_threshold ) return OutputColorRGB::white;

		// any color with a value less than this is considered black
		//if ( v < spectrum_hsv_.black_v_threshold ) return OutputColorRGB::black;

		// any color with a combined value-saturation ( v * ( 1-s ) ) greater than this is considered white
		//if ( v * ( 1.0 - s ) > spectrum_hsv_.white_combined_sv_threshold ) return OutputColorRGB::white;

		// any color with a combined value-saturation ( v * s ) less than this is considered black
		//if ( s * v < spectrum_hsv_.black_combined_sv_threshold ) return OutputColorRGB::black;

		if ( spectrum_hsv_.red.isInRange( h, true ) ) return OutputColorRGB::red;
		if ( spectrum_hsv_.orange.isInRange( h, true ) ) return OutputColorRGB::orange;
		if ( spectrum_hsv_.yellow.isInRange( h, true ) ) return OutputColorRGB::yellow;
		if ( spectrum_hsv_.green.isInRange( h, true ) ) return OutputColorRGB::green;
		if ( spectrum_hsv_.blue.isInRange( h, true ) ) return OutputColorRGB::blue;
		return OutputColorRGB::unknown;
	}

	void classifyImage()
	{
		// if the image has changed (or if the last response has yet to be generated), generate and then save a new response (and potentially a new output image)
		if ( new_img_ )
		{
			new_img_ = false;

			boost::lock_guard<boost::mutex> img_guard( img_mutex_ );
			cv_img_ = cv::Mat( bridge_.imgMsgToCv( img_ ) );
			cv::Mat hsv_img;

			for ( int y = 0; y < cv_img_.size().height; y++ )
			{
				for ( int x = 0; x < cv_img_.size().width; x++ )
				{
					// get pixel data
					cv::Vec3b pixel = cv_img_.at<cv::Vec3b> ( cv::Point( x, y ) );
					// set pixel data
					cv_img_.at<cv::Vec3b> ( cv::Point( x, y ) ) = filterColor( pixel );
				}
			}

			cv::cvtColor( cv_img_, hsv_img, CV_BGR2HLS);

			for ( int y = 0; y < hsv_img.size().height; y++ )
			{
				for ( int x = 0; x < hsv_img.size().width; x++ )
				{
					// get pixel data
					cv::Vec3b pixel = hsv_img.at<cv::Vec3b> ( cv::Point( x, y ) );
					// set pixel data
					cv_img_.at<cv::Vec3b> ( cv::Point( x, y ) ) = classifyPixel( pixel );
				}
			}
		}

		publishCvImage( cv_img_ );
	}

	void publishCvImage( cv::Mat & img )
	{
		ROS_INFO( "Publshed image" );
		IplImage * ipl_img = & ( (IplImage) img );
		img_pub_.publish( bridge_.cvToImgMsg( ipl_img ) );
	}

	void spin()
	{
		ROS_INFO( "Spinning..." );
		spinner_.spin();
	}

};

// bgr
const cv::Vec3b ColorClassifier::OutputColorRGB::red = cv::Vec3b( 0, 0, 255 );
const cv::Vec3b ColorClassifier::OutputColorRGB::orange = cv::Vec3b( 0, 128, 255 );
const cv::Vec3b ColorClassifier::OutputColorRGB::yellow = cv::Vec3b( 0, 255, 255 );
const cv::Vec3b ColorClassifier::OutputColorRGB::green = cv::Vec3b( 0, 255, 0 );
const cv::Vec3b ColorClassifier::OutputColorRGB::blue = cv::Vec3b( 255, 0, 0 );
const cv::Vec3b ColorClassifier::OutputColorRGB::black = cv::Vec3b( 0, 0, 0 );
const cv::Vec3b ColorClassifier::OutputColorRGB::white = cv::Vec3b( 255, 255, 255 );
const cv::Vec3b ColorClassifier::OutputColorRGB::unknown = cv::Vec3b( 128, 128, 128 );

int main( int argc, char ** argv )
{
	ros::init( argc, argv, "color_classifier" );
	ros::NodeHandle nh;

	ColorClassifier color_classifier( nh );
	color_classifier.spin();

	return 0;
}
